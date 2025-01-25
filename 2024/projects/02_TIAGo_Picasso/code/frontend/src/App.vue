<template>
  <div class="buttons">
    <button
      v-if="!drawingEditor && !uploadSvg"
      class="action-button back-button"
      @click="handleBackButton"
    >
      Back
    </button>
    <button
      class="action-button"
      @click="handleTest"
      v-if="!testView && (drawingEditor || uploadSvg)"
    >
      Tests
    </button>
  </div>

  <div class="app-container" v-show="drawingEditor">
    <h1>SVG Drawing Editor</h1>
    <button
      class="action-button"
      @click="
        () => {
          uploadSvg = true;
          drawingEditor = false;
        }
      "
    >
      Upload SVG
    </button>
    <div class="editor-container">
      <PaintComponent @svg="handleSvgSave" />
    </div>
  </div>
  <div class="app-container" v-if="uploadSvg">
    <h1>Upload SVG</h1>
    <button
      class="action-button"
      @click="
        () => {
          uploadSvg = false;
          drawingEditor = true;
        }
      "
    >
      Draw your SVG
    </button>
    <input type="file" accept=".svg" @change="handleFileUpload" />
  </div>
  <div class="app-container" v-if="testView">
    <h1>Test</h1>
    <SendGoalTestComponent />
  </div>
  <div class="app-container" v-show="positionDrawingInTheMap">
    <h1>Position Drawing in the Map</h1>
    <SetDrawingPositionComponent
      :mapObject="mapObject"
      :svgElement="svgElement"
      @savedSvg="handleSavedSvg"
      @svgInfo="handleSvgInfo"
      @newScaleFactor="handleNewScaleFactor"
      @sequenceOfCommands="handleSequenceOfCommands"
    />
  </div>
  <div class="app-container" v-if="monitorDrawing">
    <h1>Monitor Drawing</h1>
    <button
      class="action-button start-button"
      v-if="!processStarted"
      @click="handleStartProcess"
    >
      Start the process
    </button>
    <button class="action-button stop-button" v-else @click="handleStopProcess">
      Stop the process
    </button>
    <MapVisualizerComponent
      :mapData="mapObject"
      :scaleFactor="scaleFactor"
      :disableResizeMap="true"
      :drawingSvg="svgElementString"
      :drawingSvgWidth="svgWidth"
      :drawingSvgHeight="svgHeight"
      :drawingSvgTop="svgTop"
      :drawingSvgLeft="svgLeft"
      :drawingSvgRatio="svgWidthHeightRatio"
      :disableSvgDrag="true"
      :monitoredData="finalDrawing"
      :robotPosition="robotPosition"
      :disableSendGoal="true"
    />
  </div>
</template>

<script setup lang="ts">
/**
 * App.vue
 * This is the root component of the application.
 * It serves as the entry point for the app, handling the main views and the logic to switch between them.
 * It provides an interface to the user to draw an SVG, upload an existing SVG, test the robot's functionalities,
 * position the drawing in the map, and monitor the drawing process.
 */
import { ref, onMounted } from "vue";
import {
  getMap,
  sendCommands,
  getMonitoringData,
  stopProcess,
} from "@/api/userService";
import {
  getElementsFromSvgString,
  wrapSvgElements,
} from "@/composables/processDrawing";
import { MapObject } from "@/types/map";
import { Command } from "@/types/geometry";
import PaintComponent from "./components/PaintComponent.vue";
import SendGoalTestComponent from "./components/SendGoalTestComponent.vue";
import SetDrawingPositionComponent from "./components/SetDrawingPositionComponent.vue";
import MapVisualizerComponent from "./components/MapVisualizerComponent.vue";
import { AxiosError } from "axios";

// State variables to control the views
const drawingEditor = ref<boolean>(true);
const uploadSvg = ref<boolean>(false);
// Test view to test: map fetching, goals sending and the up/down commands sending.
const testView = ref<boolean>(false);
const positionDrawingInTheMap = ref<boolean>(false);
const monitorDrawing = ref<boolean>(false);
const processStarted = ref<boolean>(false);

// State variables to store the data
const mapObject = ref<MapObject | null>(null);
const svgElement = ref<SVGSVGElement | undefined>();
const commands = ref<Command[]>([]);

// Method to fetch the map from the backend
const handleGetMap = async () => {
  const response = await getMap();
  if (response.status === 200) {
    mapObject.value = response.data;
  } else {
    console.error("Error getting map data");
  }
};

// Fetch the map data when the component is mounted
onMounted(async () => {
  await handleGetMap();
});

// Method to handle the back button, to go back to the previous view
const handleBackButton = () => {
  if (testView.value) {
    testView.value = false;
    drawingEditor.value = true;
  } else if (positionDrawingInTheMap.value) {
    positionDrawingInTheMap.value = false;
    drawingEditor.value = true;
  } else if (monitorDrawing.value) {
    monitorDrawing.value = false;
    positionDrawingInTheMap.value = true;
  }
};

// Method to handle the SVG save event
const handleSvgSave = (svg: string | SVGSVGElement) => {
  if (svg instanceof SVGSVGElement) {
    svgElement.value = svg;
  } else if (typeof svg === "string") {
    const parser = new DOMParser();
    const doc = parser.parseFromString(svg, "image/svg+xml");
    const parsedElement = doc.documentElement;
    if (!(parsedElement instanceof SVGSVGElement)) {
      console.error("Parsed element is not a valid SVG element");
      return;
    }
    svgElement.value = parsedElement;
  } else {
    console.error("Invalid SVG type");
    return;
  }
  // Switch to the next view
  drawingEditor.value = false;
  positionDrawingInTheMap.value = true;
};

// Handle SVG Upload
const handleFileUpload = (event: Event) => {
  const fileInput = event.target as HTMLInputElement;
  const file = fileInput.files?.[0];
  if (file && file.type === "image/svg+xml") {
    const reader = new FileReader();
    reader.onload = (e) => {
      const svgContent = e.target?.result as string;
      const parser = new DOMParser();
      const doc = parser.parseFromString(svgContent, "image/svg+xml");
      // Get the first <svg> element
      const parsedElement = doc.querySelector("svg");
      if (!(parsedElement instanceof SVGSVGElement)) {
        console.error("Parsed element is not a valid SVG element");
        return;
      }
      const serializer = new XMLSerializer();
      const svgString = serializer.serializeToString(parsedElement);
      const acceptedSvgElements = getElementsFromSvgString(svgString);
      const wrappedSvgElement = wrapSvgElements(acceptedSvgElements);
      svgElement.value = wrappedSvgElement;
      uploadSvg.value = false;
      positionDrawingInTheMap.value = true;
    };
    reader.onerror = () => {
      console.error("Error reading the file.");
    };
    reader.readAsText(file);
  } else {
    console.error("Please upload a valid SVG file.");
  }
};

// Handle switch to test view
const handleTest = () => {
  drawingEditor.value = false;
  uploadSvg.value = false;
  testView.value = true;
};

// Variables to position the drawing in the map
const svgElementString = ref<string | undefined>();
const svgWidth = ref<number | undefined>();
const svgHeight = ref<number | undefined>();
const svgWidthHeightRatio = ref<number | undefined>();
const svgTop = ref<number | undefined>();
const svgLeft = ref<number | undefined>();
const scaleFactor = ref<number | undefined>();

// Methods to handle the events from the SetDrawingPositionComponent
const handleSavedSvg = (svgString: string) => {
  svgElementString.value = svgString;
};

// Update the svg info when the svg is dragged or resized
const handleSvgInfo = (svgInfo: {
  width: number;
  height: number;
  top: number;
  left: number;
}) => {
  svgWidth.value = svgInfo.width;
  svgHeight.value = svgInfo.height;
  svgTop.value = svgInfo.top;
  svgLeft.value = svgInfo.left;
  svgWidthHeightRatio.value = svgInfo.width / svgInfo.height;
};

// Update the scale factor of the map when the map is resized
const handleNewScaleFactor = (newScaleFactor: number) => {
  scaleFactor.value = newScaleFactor;
};

// When the user has finished positioning the drawing in the map and clicks draw it
// the SetDrawingPosition component will emit the sequence of commands to draw the SVG.
const handleSequenceOfCommands = (sequenceOfCommands: Command[]) => {
  commands.value = sequenceOfCommands;
  positionDrawingInTheMap.value = false;
  monitorDrawing.value = true;
};

// Polling interval to fetch the monitoring data, identifier to clear the interval
const pollingInterval = ref<number | null>(null);
// Final drawing to display on the map containing the sequence of points where the pen was down
const finalDrawing = ref<{ x: number; y: number }[]>([]);
// Latest Robot position to display on the map
const robotPosition = ref<{ x: number; y: number }>({ x: 0, y: 0 });

// Method to start the process of drawing the SVG
const handleStartProcess = () => {
  processStarted.value = true;
  finalDrawing.value = [];
  console.log("Start the process");

  // Clear any previous polling
  if (pollingInterval.value) {
    clearInterval(pollingInterval.value);
  }

  // Send the sequence of commands to the backend
  sendCommands(commands.value)
    .then(() => {
      // Start polling for monitoring data
      pollingInterval.value = window.setInterval(async () => {
        try {
          const response = await getMonitoringData();
          if (response.data.length === 0) {
            return;
          }
          // Update robot position to the latest position
          robotPosition.value =
            response.data[response.data.length - 1].position;
          // Add the points where the pen was down to the final drawing
          finalDrawing.value.push(
            ...response.data
              .filter((data) => data.pen_state === "down")
              .map((data) => ({ x: data.position.x, y: data.position.y }))
          );
        } catch (error) {
          if (error instanceof AxiosError) {
            // This error is thrown when the monitoring service has been stopped at the backend level
            if (error.response?.status === 503) {
              console.log("Monitoring service has been stopped");
            }
          } else {
            console.error("Error fetching monitoring data:", error);
          }
          // Stop the process if the polling fails
          if (pollingInterval.value) {
            handleStopProcess();
          }
        }
      }, 1000); // Poll every 1s
    })
    .catch((error) => {
      console.error("Error starting the process:", error);
    });
};

// Method to stop the process of drawing the SVG
const handleStopProcess = async () => {
  processStarted.value = false;
  console.log("Stop the process");
  // Stop polling
  if (pollingInterval.value) {
    clearInterval(pollingInterval.value);
    pollingInterval.value = null;
  }

  // Call the backend to stop the process
  try {
    const response = await stopProcess();
    console.log("Backend response for stopping process:", response.data);
  } catch (error) {
    console.error("Failed to stop the process:", error);
  }
};
</script>

<style>
#app {
  font-family: Avenir, Helvetica, Arial, sans-serif;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
  color: #2c3e50;
  text-align: center;
}
.app-container {
  width: 100%;
  height: 100%;
  display: flex;
  flex-direction: column;
  align-items: center;
  margin: auto;
}

.editor-container {
  width: 50vw;
  height: 80vh;
  position: relative; /* For any absolute positioning inside */
}

.action-button {
  border-radius: 0.5rem;
  border: 1px solid #000000;
  padding: 0.5rem 1rem;
  background-color: #00b7ff;
  margin: 0.5rem;
}

.action-button:hover {
  background-color: #0099e6;
}

.start-button {
  flex-grow: 0.2;
  border-radius: 0.5rem;
  border: 1px solid #000000;
  padding: 0.5rem 1rem;
  background-color: #00ff40;
}

.start-button:hover {
  background-color: #00e639;
}

.stop-button {
  flex-grow: 0.2;
  border-radius: 0.5rem;
  border: 1px solid #000000;
  padding: 0.5rem 1rem;
  background-color: #ff0000;
}

.stop-button:hover {
  background-color: #e60000;
}
</style>
