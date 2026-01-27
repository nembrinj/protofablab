import time
import numpy as np
import pandas as pd
import argparse
from pathlib import Path
import joblib
from brainflow.board_shim import BoardShim, BrainFlowInputParams, BoardIds
from brainflow.data_filter import DataFilter, FilterTypes
from collections import deque
import matplotlib.pyplot as plt
import paho.mqtt.client as mqtt
import json

try:
    import torch
    import torch.nn as nn
    import torch.optim as optim
    from torch.utils.data import DataLoader, TensorDataset
    PYTORCH_AVAILABLE = True
except ImportError:
    PYTORCH_AVAILABLE = False
    print("WARNING: PyTorch not available.")

from sklearn.model_selection import train_test_split
from sklearn.preprocessing import LabelEncoder
from sklearn.metrics import confusion_matrix, classification_report

BOARD_ID = BoardIds.MUSE_2_BOARD.value
SAMPLING_RATE = BoardShim.get_sampling_rate(BOARD_ID)
EEG_CHANNEL_NAMES = BoardShim.get_eeg_names(BOARD_ID)
EEG_CHANNELS_LIST = BoardShim.get_eeg_channels(BOARD_ID)
WINDOW_SEC = 2
WINDOW_SAMPLES = int(WINDOW_SEC * SAMPLING_RATE)
BASE_DATA_DIR = Path("eeg_data")
BASE_MODELS_DIR = Path("models")

if PYTORCH_AVAILABLE:
    if torch.cuda.is_available():
        DEVICE = "cuda"
    elif torch.backends.mps.is_available():
        DEVICE = "mps"
    else:
        DEVICE = "cpu"
else:
    DEVICE = "cpu"

class EEGNet(nn.Module):
    def __init__(self, nb_classes, Chans=4, Samples=WINDOW_SAMPLES,
                 dropoutRate=0.5, kernLength=64, F1=8,
                 D=2, F2=16, norm_rate=0.25):
        super(EEGNet, self).__init__()

        # Block 1
        self.block1 = nn.Sequential(
            nn.Conv2d(1, F1, (1, kernLength), padding=(0, kernLength // 2), bias=False),
            nn.BatchNorm2d(F1),
            nn.Conv2d(F1, F1 * D, (Chans, 1), groups=F1, bias=False),
            nn.BatchNorm2d(F1 * D),
            nn.ELU(),
            nn.AvgPool2d((1, 4)),
            nn.Dropout(dropoutRate)
        )

        # Block 2
        self.block2 = nn.Sequential(
            nn.Conv2d(F1 * D, F1 * D, (1, 16), padding=(0, 16 // 2), groups=F1 * D, bias=False),
            nn.Conv2d(F1 * D, F2, (1, 1), bias=False),
            nn.BatchNorm2d(F2),
            nn.ELU(),
            nn.AvgPool2d((1, 8)),
            nn.Dropout(dropoutRate)
        )

        # Classifier
        demo_input = torch.randn(1, 1, Chans, Samples)
        with torch.no_grad():
            flattened_size = self._get_flattened_size(demo_input)
        
        self.classifier = nn.Sequential(
            nn.Flatten(),
            nn.Linear(flattened_size, nb_classes)
        )

    def _get_flattened_size(self, x):
        x = self.block1(x)
        x = self.block2(x)
        return x.view(1, -1).size(1)

    def forward(self, x):
        x = self.block1(x)
        x = self.block2(x)
        x = self.classifier(x)
        return x


def record(user_id, label, duration=30):
    params = BrainFlowInputParams()
    board = BoardShim(BOARD_ID, params)

    data_dir = BASE_DATA_DIR / user_id
    rejected_dir = data_dir / "rejected"
    data_dir.mkdir(parents=True, exist_ok=True)
    rejected_dir.mkdir(exist_ok=True)
    
    try:
        board.prepare_session()
        board.start_stream(45000 * 2)
        
        print(f"\nRecording '{label.upper()}' in 5 seconds...")
        for i in range(5, 0, -1):
            print(f"{i}...")
            time.sleep(1)

        print(f"RECORDING '{label.upper()}' for {duration}s...")
        for i in range(duration):
            time.sleep(1)
            board.get_current_board_data(1) 
            print(f"{i+1}/{duration}s", end='\r')
        
        data = board.get_board_data()
        
    finally:
        if board.is_prepared():
            print("\nRecording complete")
            board.stop_stream()
            board.release_session()

    eeg_data = data[EEG_CHANNELS_LIST]
    ARTIFACT_THRESHOLD_STD = 80.0 
    
    is_noisy = False
    for i in range(eeg_data.shape[0]):
        channel_std = np.std(eeg_data[i])
        if channel_std > ARTIFACT_THRESHOLD_STD:
            print(f"WARNING: Channel {EEG_CHANNEL_NAMES[i]} exceeds threshold ({channel_std:.1f} > {ARTIFACT_THRESHOLD_STD})")
            is_noisy = True
            
    timestamp = int(time.time())
    
    if is_noisy:
        print("Recording NOISY - saved to 'rejected' folder")
        file_path = rejected_dir / f"{label}_{timestamp}.csv"
    else:
        print("Recording CLEAN")
        file_path = data_dir / f"{label}_{timestamp}.csv"
        
    df_to_save = pd.DataFrame(np.transpose(eeg_data))
    df_to_save.columns = EEG_CHANNEL_NAMES
    df_to_save.to_csv(file_path, index=False)
    print(f"Saved {eeg_data.shape[1]} samples to {file_path}")

def load_and_augment_data(user_id):
    data_dir = BASE_DATA_DIR / user_id
    if not data_dir.exists() or not any(data_dir.glob('*.csv')):
        print(f"Error: Data directory '{data_dir}' is empty or does not exist.")
        return None, None, None

    features, labels = [], []
    stride = int(0.25 * SAMPLING_RATE)
    noise_factor = 0.05

    print("Loading and preprocessing data...")
    for file_path in data_dir.rglob("*.csv"):
        if 'rejected' in file_path.parts: 
            continue
        
        label = file_path.stem.split('_')[0]
        data_df = pd.read_csv(file_path)
        data = np.transpose(data_df[EEG_CHANNEL_NAMES].values).copy()

        for i in range(0, data.shape[1] - WINDOW_SAMPLES, stride):
            window = data[:, i:i + WINDOW_SAMPLES].copy()
            
            if np.random.rand() > 0.5:
                aug_type = np.random.randint(0, 3)
                if aug_type == 0: 
                    window += np.random.normal(0, np.std(window) * noise_factor, window.shape)
                elif aug_type == 1: 
                    window *= -1
                elif aug_type == 2: 
                    window *= np.random.uniform(0.9, 1.1)

            for ch in range(window.shape[0]):
                DataFilter.perform_bandpass(window[ch], SAMPLING_RATE, 1.0, 45.0, 4, FilterTypes.BUTTERWORTH_ZERO_PHASE, 0)
                DataFilter.perform_bandstop(window[ch], SAMPLING_RATE, 48.0, 52.0, 4, FilterTypes.BUTTERWORTH_ZERO_PHASE, 0)
            
            features.append(window)
            labels.append(label)

    X = np.array(features)
    le = LabelEncoder()
    y = le.fit_transform(labels)
    X = X.reshape(X.shape[0], 1, X.shape[1], X.shape[2])

    print(f"Total samples: {X.shape[0]}")
    return X, y, le

def train(user_id):
    if not PYTORCH_AVAILABLE:
        print("ERROR: PyTorch not available")
        return

    user_model_dir = BASE_MODELS_DIR / user_id
    user_model_dir.mkdir(parents=True, exist_ok=True)
    
    X, y, le = load_and_augment_data(user_id)
    if X is None: 
        return

    joblib.dump(le, user_model_dir / "label_encoder_eegnet_4.pkl")
    
    X_train_val, X_test, y_train_val, y_test = train_test_split(
        X, y, test_size=0.2, random_state=42, stratify=y
    )
    X_train, X_val, y_train, y_val = train_test_split(
        X_train_val, y_train_val, test_size=0.2, random_state=42, stratify=y_train_val
    )
    
    train_loader = DataLoader(
        TensorDataset(torch.from_numpy(X_train).float(), torch.from_numpy(y_train).long()), 
        batch_size=32, shuffle=True
    )
    val_loader = DataLoader(
        TensorDataset(torch.from_numpy(X_val).float(), torch.from_numpy(y_val).long()), 
        batch_size=32
    )
    test_loader = DataLoader(
        TensorDataset(torch.from_numpy(X_test).float(), torch.from_numpy(y_test).long()), 
        batch_size=32
    )
    
    model = EEGNet(
        nb_classes=len(le.classes_), 
        Chans=X_train.shape[2], 
        Samples=X_train.shape[3], 
        dropoutRate=0.75
    ).to(DEVICE)
    
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.parameters(), lr=0.001, weight_decay=1e-4)

    best_val_accuracy = 0
    patience_counter = 0
    patience = 10
    
    print(f"\nTraining EEGNet on device: {DEVICE}")
    for epoch in range(100):
        model.train()
        for inputs, labels in train_loader:
            inputs, labels = inputs.to(DEVICE), labels.to(DEVICE)
            optimizer.zero_grad()
            outputs = model(inputs)
            loss = criterion(outputs, labels)
            loss.backward()
            optimizer.step()
        
        model.eval()
        val_correct, val_total = 0, 0
        with torch.no_grad():
            for inputs, labels in val_loader:
                inputs, labels = inputs.to(DEVICE), labels.to(DEVICE)
                outputs = model(inputs)
                _, predicted = torch.max(outputs.data, 1)
                val_total += labels.size(0)
                val_correct += (predicted == labels).sum().item()
        
        val_accuracy = 100 * val_correct / val_total
        print(f'Epoch {epoch+1:02d}.. Val Accuracy: {val_accuracy:.2f}%')

        if val_accuracy > best_val_accuracy:
            best_val_accuracy = val_accuracy
            patience_counter = 0
            torch.save(model.state_dict(), user_model_dir / "emotion_model_eegnet_4.pth")
        else:
            patience_counter += 1
        
        if patience_counter >= patience:
            print(f"Early stopping at epoch {epoch+1}")
            break
    
    print("\nTesting on final data...")
    model.load_state_dict(torch.load(user_model_dir / "emotion_model_eegnet_4.pth"))
    model.eval()
    
    all_predictions = []
    all_labels = []
    
    with torch.no_grad():
        for inputs, labels in test_loader:
            inputs, labels = inputs.to(DEVICE), labels.to(DEVICE)
            outputs = model(inputs)
            _, predicted = torch.max(outputs.data, 1)
            all_predictions.extend(predicted.cpu().numpy())
            all_labels.extend(labels.cpu().numpy())
    
    all_predictions = np.array(all_predictions)
    all_labels = np.array(all_labels)
    final_accuracy = 100 * np.sum(all_predictions == all_labels) / len(all_labels)
    
    print(f"\nFINAL ACCURACY: {final_accuracy:.2f}%")
    
    cm = confusion_matrix(all_labels, all_predictions)
    class_names = le.classes_
    
    print(f"\n{'':>12}", end="")
    for name in class_names:
        print(f"{name:>12}", end="")
    print()
    
    for i, name in enumerate(class_names):
        print(f"{name:>12}", end="")
        for j in range(len(class_names)):
            print(f"{cm[i,j]:>12}", end="")
        print()
    
    print()
    for i, class_name in enumerate(class_names):
        class_mask = all_labels == i
        class_correct = np.sum((all_predictions == all_labels) & class_mask)
        class_total = np.sum(class_mask)
        class_accuracy = 100 * class_correct / class_total if class_total > 0 else 0
        print(f"{class_name.upper():>10}: {class_accuracy:>6.2f}%")
    
    print(f"\nModel saved to {user_model_dir / 'emotion_model_eegnet_4.pth'}")

def predict(user_id, mqtt_ip=None):
    if not PYTORCH_AVAILABLE:
        print("ERROR: PyTorch not available")
        return
        
    print(f"Using device: {DEVICE}")
    
    mqtt_connected = False
    client = None
    
    if mqtt_ip:
        MQTT_TOPIC = "/set_state"
        client = mqtt.Client()
        
        try:
            client.connect(mqtt_ip, 1883, 60)
            client.loop_start() 
            print("MQTT Connected")
            mqtt_connected = True
        except Exception as e:
            print(f"MQTT Failed: {e}")
    else:
        print("Running without MQTT")

    user_model_dir = BASE_MODELS_DIR / user_id
    le_path = user_model_dir / "label_encoder_eegnet_4.pkl"
    model_path = user_model_dir / "emotion_model_eegnet_4.pth"
    
    if not le_path.exists():
        print(f"Error: Label encoder not found")
        return
    
    if not model_path.exists():
        print(f"Error: Model not found")
        return
    
    le = joblib.load(le_path)
    n_classes = len(le.classes_)
    
    model = EEGNet(nb_classes=n_classes, Chans=4, Samples=WINDOW_SAMPLES).to(DEVICE)
    model.load_state_dict(torch.load(model_path, map_location=DEVICE))
    model.eval()
    print("Model loaded")

    params = BrainFlowInputParams()
    board = BoardShim(BOARD_ID, params)

    prediction_buffer = deque(maxlen=10)
    state_threshold = 0.7
    current_stable_state = "neutral"
    
    seconds_to_show = 5
    buffer_size = seconds_to_show * SAMPLING_RATE
    data_buffers = np.zeros((len(EEG_CHANNELS_LIST), buffer_size))
    
    plt.ion()
    fig, axs = plt.subplots(len(EEG_CHANNELS_LIST), 1, figsize=(12, 8), sharex=True)
    lines = [ax.plot(np.zeros(buffer_size))[0] for ax in axs]
    
    for i, ax in enumerate(axs):
        ax.set_title(EEG_CHANNEL_NAMES[i])
        ax.set_ylim(-200, 200)
        ax.grid(True)
    axs[-1].set_xlabel('Time (s)')
    fig.text(0.06, 0.5, 'Voltage (uV)', va='center', rotation='vertical')
    
    time_axis = np.arange(-seconds_to_show, 0, 1.0/SAMPLING_RATE)

    color_map = {
        'calm': '#3498db',
        'focused': '#f39c12',
        'happy': '#2ecc71',
        'tense': '#e74c3c',
        'neutral': '#95a5a6'
    }

    last_prediction_time = time.time()
    prediction_interval = WINDOW_SEC
    prediction_label = "INIT"
    confidence = 0.0

    try:
        board.prepare_session()
        board.start_stream(45000)
        time.sleep(2)
        print("Starting live prediction...")
        
        while True:
            new_data = board.get_board_data()
            
            if new_data.shape[1] > 0:
                new_eeg_data = new_data[EEG_CHANNELS_LIST]
                num_new_samples = new_eeg_data.shape[1]
                
                data_buffers = np.roll(data_buffers, -num_new_samples, axis=1)
                data_buffers[:, -num_new_samples:] = new_eeg_data
                
                viz_buffers = data_buffers.copy()
                for ch in range(viz_buffers.shape[0]):
                    DataFilter.perform_bandpass(viz_buffers[ch], SAMPLING_RATE, 1.0, 45.0, 4, FilterTypes.BUTTERWORTH_ZERO_PHASE, 0)
                    DataFilter.perform_bandstop(viz_buffers[ch], SAMPLING_RATE, 48.0, 52.0, 4, FilterTypes.BUTTERWORTH_ZERO_PHASE, 0)

                for i, line in enumerate(lines):
                    line.set_ydata(viz_buffers[i])
                    if line.get_xdata().shape[0] != time_axis.shape[0]:
                        line.set_xdata(time_axis)
                
                for ax in axs:
                    ax.relim()
                    ax.autoscale_view(scalex=False, scaley=True)


                if mqtt_connected:
                    new_filtered_data = viz_buffers[:, -num_new_samples:]
                    
                    eeg_payload = {
                        name: new_filtered_data[i].tolist() 
                        for i, name in enumerate(EEG_CHANNEL_NAMES)
                    }
                    client.publish("/eeg", json.dumps(eeg_payload))

                if (time.time() - last_prediction_time) > prediction_interval:
                    last_prediction_time = time.time()
                    
                    analysis_window = np.ascontiguousarray(data_buffers[:, -WINDOW_SAMPLES:])
                    
                    if analysis_window.shape[1] == WINDOW_SAMPLES:
                        for ch in range(analysis_window.shape[0]):
                            DataFilter.perform_bandpass(analysis_window[ch], SAMPLING_RATE, 1.0, 45.0, 4, FilterTypes.BUTTERWORTH_ZERO_PHASE, 0)
                            DataFilter.perform_bandstop(analysis_window[ch], SAMPLING_RATE, 48.0, 52.0, 4, FilterTypes.BUTTERWORTH_ZERO_PHASE, 0)

                        input_tensor = torch.tensor(analysis_window, dtype=torch.float32).reshape(1, 1, 4, WINDOW_SAMPLES).to(DEVICE)
                        with torch.no_grad():
                            outputs = model(input_tensor)
                            probs = torch.softmax(outputs, dim=1).cpu().numpy()[0]
                            pred_idx = np.argmax(probs)
                            prediction_label = le.inverse_transform([pred_idx])[0]
                            confidence = probs[pred_idx]


                        prediction_buffer.append(prediction_label)
                        if len(prediction_buffer) == prediction_buffer.maxlen:
                            unique_labels, counts = np.unique(list(prediction_buffer), return_counts=True)
                            majority_label = unique_labels[np.argmax(counts)]
                            majority_percent = np.max(counts) / prediction_buffer.maxlen

                            if majority_label != current_stable_state and majority_percent >= state_threshold:
                                current_stable_state = majority_label
                                
                                if mqtt_connected:
                                    payload = json.dumps({
                                        "state": current_stable_state,
                                        "confidence": float(confidence)
                                    })
                                    client.publish(MQTT_TOPIC, payload)
                                    print(f"\n>>> STATE: {current_stable_state.upper()} ({confidence*100:.0f}%)")
                                else:
                                    print(f"\n>>> STATE: {current_stable_state.upper()} ({confidence*100:.0f}%)")

                buffer_viz = " ".join([l[:3].upper() for l in prediction_buffer])
                title_str = (
                    f"ROBOT STATE: {current_stable_state.upper()}\n"
                    f"Buffer: [{buffer_viz}] | Raw: {prediction_label.upper()} ({confidence*100:.0f}%)"
                )
                fig.suptitle(
                    title_str,
                    fontsize=16,
                    fontweight='bold',
                    color=color_map.get(current_stable_state, '#000000')
                )

                plt.pause(0.001)
            else:
                time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nStopping script...")
    finally:
        if board.is_prepared():
            board.stop_stream()
            board.release_session()
        if mqtt_connected and client:
            client.loop_stop()
            client.disconnect()
        plt.ioff()
        plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="EEG Emotion Classifier (EEGNet)")
    subparsers = parser.add_subparsers(dest='mode', required=True)

    record_parser = subparsers.add_parser('record')
    record_parser.add_argument('user', type=str, help='User ID (yk or ab)')
    record_parser.add_argument('label', type=str, help='Emotion to record (happy, calm, stressed, focused)')
    record_parser.add_argument('--duration', type=int, default=30, help='Duration of CSV recording in seconds')
    record_parser.add_argument('--count', type=int, default=1, help='Number of consecutive recordings for the same state')

    train_parser = subparsers.add_parser('train')
    train_parser.add_argument('user', type=str, help='User ID (yk or ab)')

    predict_parser = subparsers.add_parser('predict')
    predict_parser.add_argument('user', type=str, help='User ID (yk or ab)')
    predict_parser.add_argument('--mqtt_ip', type=str, default="192.168.1.13", help='IP address of computer running Node-RED and MQTT')
    
    args = parser.parse_args()
    
    if args.mode == 'record':
        for i in range(args.count):
            if args.count > 1:
                print(f"\n=== Session {i+1}/{args.count} ===")
            
            record(args.user, args.label, args.duration)
            
            if i < args.count - 1:
                print("\nWaiting 5s...")
                time.sleep(5)
                
    elif args.mode == 'train':
        train(args.user)
        
    elif args.mode == 'predict':
        predict(args.user, args.mqtt_ip)
