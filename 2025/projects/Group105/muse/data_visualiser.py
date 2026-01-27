import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

base_folder = Path("eeg_data")
subject_folder = "yk"
data_dir = base_folder / subject_folder

combined_output_dir = Path(f"plots_{subject_folder}_combined")
separate_output_dir = Path(f"plots_{subject_folder}_separate")
combined_output_dir.mkdir(exist_ok=True)
separate_output_dir.mkdir(exist_ok=True)

if not data_dir.is_dir():
    print(f"Error: Directory not found at {data_dir.resolve()}")
else:
    print(f"Processing files in: {data_dir.resolve()}")
    print(f"Saving combined plots to: {combined_output_dir.resolve()}")
    print(f"Saving separate plots to: {separate_output_dir.resolve()}")

    csv_files = list(data_dir.glob("*.csv"))

    if not csv_files:
        print("No .csv files found in this directory.")
    
    for file_path in csv_files:
        try:
            filename_stem = file_path.stem
            emotion = filename_stem.split('_')[0].capitalize()
            df = pd.read_csv(file_path)

            if df.empty:
                print(f"Skipping empty file: {file_path.name}")
                continue

            plot_filename = f"{filename_stem}.png"
            
            # 1. Generate combined plot (all 4 channels on same graph)
            plt.figure(figsize=(15, 7))
            df.plot(ax=plt.gca())
            plt.title(f"Subject: {subject_folder} | Emotion: {emotion}\n(File: {file_path.name})", fontsize=16)
            plt.xlabel("Sample Index", fontsize=12)
            plt.ylabel("EEG Value", fontsize=12)
            plt.legend(loc='upper right')
            plt.grid(True, which='both', linestyle='--', linewidth=0.5)
            plt.savefig(combined_output_dir / plot_filename)
            plt.close()

            # 2. Generate separate plot (each channel as subplot)
            channels = df.columns
            num_channels = len(channels)
            fig_height = num_channels * 2.5
            
            fig, axes = plt.subplots(num_channels, 1, figsize=(15, fig_height), sharex=True)
            fig.suptitle(f"Subject: {subject_folder} | Emotion: {emotion}\n(File: {file_path.name})", fontsize=16, y=1.02)
            
            if num_channels == 1:
                axes = [axes]
            
            for i, channel in enumerate(channels):
                df[channel].plot(ax=axes[i])
                axes[i].set_ylabel(channel, fontsize=12, weight='bold')
                axes[i].grid(True, which='both', linestyle='--', linewidth=0.5)
            
            axes[-1].set_xlabel("Sample Index", fontsize=12)
            plt.tight_layout(rect=[0, 0.03, 1, 0.98])
            fig.savefig(separate_output_dir / plot_filename)
            plt.close(fig)

            print(f"Saved plots for: {file_path.name}")

        except Exception as e:
            print(f"Could not process file {file_path.name}: {e}")

    print(f"Combined plots: '{combined_output_dir.name}'")
    print(f"Separate plots: '{separate_output_dir.name}'")