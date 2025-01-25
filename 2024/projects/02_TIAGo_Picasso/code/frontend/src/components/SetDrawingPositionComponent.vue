<template>
  <div class="buttons">
    <button class="action-button" @click="handleGetMap">Get Map</button>
    <button
      class="action-button"
      v-if="pointsPerElement.length > 0"
      @click="handleDrawIt"
    >
      Draw it
    </button>
  </div>
  <MapVisualizerComponent
    v-if="mapObject"
    :mapData="mapObject"
    :scaleFactor="scaleFactor"
    :disableSendGoal="true"
    :drawingSvg="savedSvg"
    :drawingSvgWidth="svgWidthNumber"
    :drawingSvgHeight="svgHeight"
    :drawingSvgRatio="svgWidthHeightRatio"
    :drawingSvgTop="svgTop"
    :drawingSvgLeft="svgLeft"
    @canvasSize="handleCanvasSize"
    @newScaleFactor="handleNewScaleFactor"
    @svgInfo="handleSvgInfo"
  />
  <div class="drawing-dim-container">
    Drawing Width:
    <input
      type="number"
      v-model="svgWidth"
      :min="0"
      :max="maxSvgWidth"
      :step="10"
      class=""
      @input="handleSvgWidthInput"
    />
    Drawing Height:
    {{ svgHeight }}
  </div>

  <div class="slider-container">
    <label for="pointDistanceSlider">Select Point Distance:</label>
    <input
      id="pointDistanceSlider"
      type="range"
      v-model="pointDistance"
      :min="minPointDistance"
      :max="maxPointDistance"
      :step="1"
    />
    Selected Distance:
    <input
      type="number"
      :min="minPointDistance"
      :max="maxPointDistance"
      class="point-distance-input"
      v-model="pointDistance"
    />
  </div>
  <div class="preview-container">
    <!-- Left: SVG Preview -->
    <div class="preview-left">
      <div
        v-html="savedSvgInContainer"
        class="svg-container"
        :style="{ width: width + 'px', height: height + 'px' }"
      ></div>
    </div>
    <!-- Right: Points Visualization -->
    <div class="preview-right">
      <SetOfPointsVisualizerComponent
        :points="allPoints"
        :canvasWidth="width"
        :canvasHeight="height"
      />
    </div>
  </div>
</template>
<script setup lang="ts">
/**
 * SetDrawingPositionComponent.vue
 * It allows to set the drawing position on the map and choose the distance between points in the drawing.
 */
import { ref, watch, onMounted, onUnmounted, computed } from "vue";
import SetOfPointsVisualizerComponent from "./SetOfPointsVisualizerComponent.vue";
import MapVisualizerComponent from "./MapVisualizerComponent.vue";
import { MapObject } from "@/types/map";
import { Command } from "@/types/geometry";
import {
  getElementsFromSvgString,
  getPointsFromElement,
  normalizeElementPointsToCanvas,
  canvasToMapCoordinates,
  fromArrowToPose,
} from "@/composables/processDrawing";
import { getMap } from "@/api/userService";

// Define the props of the component
const props = defineProps<{
  svgElement: SVGSVGElement | undefined;
  mapObject: MapObject | null;
}>();

// Define the emits of the component
const emit = defineEmits<{
  (event: "sequenceOfCommands", sequenceOfCommands: Command[]): void;
  (event: "savedSvg", savedSvg: string): void;
  (
    event: "svgInfo",
    svgInfo: { width: number; height: number; top: number; left: number }
  ): void;
  (event: "newScaleFactor", newScaleFactor: number): void;
}>();

const mapObject = ref<MapObject | null>(props.mapObject);
const scaleFactor = ref<number>(2);
const savedSvg = ref<string>("");
// SVG set by the user
const svgWidth = ref<number | undefined>();
// Width of the SVG drawing as a number
const svgWidthNumber = ref<number>();
const svgHeight = ref<number | undefined>();
const svgWidthHeightRatio = ref<number | undefined>();
const svgTop = ref<number | undefined>();
const svgLeft = ref<number | undefined>();

const canvasWidth = ref<number | undefined>();
const canvasHeight = ref<number | undefined>();
const maxSvgWidth = ref<number>();

const pointDistance = ref<number>(20);
const minPointDistance = 5;
const maxPointDistance = 200;

// SVG drawing shown in a container
const savedSvgInContainer = ref<string>("");
const width = ref<number>(300);
const height = ref<number>(300);

// Set of points for each SVG element in the drawing
const pointsPerElement = ref<
  { closed: boolean; points: { x: number; y: number }[] }[]
>([]);

// All points in the drawing
const allPoints = computed<{ x: number; y: number }[]>(() => {
  return pointsPerElement.value.map((element) => element.points).flat();
});

// Component mounted hook, gets the map from the backend and sets the SVG element and process
// it with the set point distance.
onMounted(async () => {
  await handleGetMap();
  handleSvgElement();
  processDrawing();
});

onUnmounted(() => {
  // Reset the svgDrawingWidth
  svgWidthNumber.value = undefined;
});

// Watch for changes in the SVG element and in case reprocess it
watch(
  () => props.svgElement,
  () => {
    handleSvgElement();
    processDrawing();
  }
);

// Watch for changes in the point distance if it is within the limits, reprocess the drawing
watch(pointDistance, () => {
  if (
    pointDistance.value > minPointDistance &&
    pointDistance.value < maxPointDistance
  ) {
    processDrawing();
  }
});

// Method to handle the canvas size, sets maxSvgWidth and svgWidth based on the canvas size
const handleCanvasSize = (size: { width: number; height: number }) => {
  canvasWidth.value = size.width;
  canvasHeight.value = size.height;
  if (!svgWidthHeightRatio.value) {
    console.error("SVG width height ratio not set");
    return;
  }
  if (svgWidthHeightRatio.value > 1) {
    maxSvgWidth.value = canvasWidth.value;
  } else {
    maxSvgWidth.value = Math.floor(
      canvasHeight.value * svgWidthHeightRatio.value
    );
  }
  svgWidth.value = Math.min(svgWidthNumber.value || 0, maxSvgWidth.value);
  svgWidthNumber.value = svgWidth.value;
};

// Method to handle the SVG info, sets the SVG width, height, top and left
const handleSvgInfo = (svgInfo: {
  width: number;
  height: number;
  top: number;
  left: number;
}) => {
  svgWidth.value = svgInfo.width;
  svgWidthNumber.value = svgWidth.value;
  svgHeight.value = svgInfo.height;
  svgTop.value = svgInfo.top;
  svgLeft.value = svgInfo.left;
  emit("svgInfo", svgInfo);
};

// Method to handle a change in the SVG width input done by the user.
// To avoid possible overflow top and left are set to 0, top-left corner of the canvas.
const handleSvgWidthInput = (event: Event) => {
  const value = (event.target as HTMLInputElement).value;
  svgWidth.value =
    value === "" ? 0 : Math.min(Number(value), maxSvgWidth.value || 0);
  svgWidthNumber.value = svgWidth.value;
  svgTop.value = 0;
  svgLeft.value = 0;
};

// Fetch the map from the backend
const handleGetMap = async () => {
  const maxRetries = 3;
  const delay = (ms: number) =>
    new Promise((resolve) => setTimeout(resolve, ms));
  let attempt = 0;

  while (attempt < maxRetries) {
    try {
      const response = await getMap();
      if (response.status === 200) {
        mapObject.value = response.data;
        return;
      } else {
        console.error(`Error getting map data. Status: ${response.status}`);
      }
    } catch (error) {
      console.error(`Attempt ${attempt + 1} failed. Error:`, error);
    }

    attempt++;
    if (attempt < maxRetries) {
      console.log(`Retrying... (${attempt}/${maxRetries})`);
      await delay(1000);
    }
  }
  console.error("All retry attempts failed.");
};

// Method to handle a new scale factor of the map
const handleNewScaleFactor = (newScaleFactor: number) => {
  scaleFactor.value = newScaleFactor;
  emit("newScaleFactor", newScaleFactor);
};

// Method to handle the SVG element
const handleSvgElement = () => {
  if (props.svgElement) {
    props.svgElement.style.visibility = "hidden";
    // To calculate the bounding box of the SVG, we need to append it to the DOM
    document.body.appendChild(props.svgElement);
    const bbox = props.svgElement.getBBox();
    svgWidthHeightRatio.value = bbox.width / bbox.height;
    // Remove the element from the DOM
    props.svgElement.remove();
    // Reshow the element again
    props.svgElement.style.visibility = "visible";
    const { x, y, width: widthBBox, height: heightBBox } = bbox;
    // Set the max width based on the canvas width
    if (!canvasWidth.value || !canvasHeight.value) {
      console.error("Canvas width or height not set");
      handleGetMap();
      return;
    }
    if (svgWidthHeightRatio.value > 1) {
      maxSvgWidth.value = canvasWidth.value;
    } else {
      maxSvgWidth.value = Math.floor(
        canvasHeight.value * svgWidthHeightRatio.value
      );
    }
    props.svgElement.setAttribute(
      "viewBox",
      `${x} ${y} ${widthBBox} ${heightBBox}`
    );
    props.svgElement.classList.add("no-border");

    // Serialize back to a string for saving
    const serializer = new XMLSerializer();
    savedSvg.value = serializer.serializeToString(props.svgElement);

    emit("savedSvg", savedSvg.value);

    // Calculate scaled dimensions to fit within the container
    const containerWidth = width.value;
    const containerHeight = height.value;

    let scaledWidth, scaledHeight;

    if (svgWidthHeightRatio.value > 1) {
      // Landscape: scale based on width
      scaledWidth = containerWidth;
      scaledHeight = containerWidth / svgWidthHeightRatio.value;
    } else {
      // Portrait: scale based on height
      scaledHeight = containerHeight;
      scaledWidth = containerHeight * svgWidthHeightRatio.value;
    }

    // Limit scaled dimensions to fit within the container
    if (scaledWidth > containerWidth) {
      scaledWidth = containerWidth;
      scaledHeight = containerWidth / svgWidthHeightRatio.value;
    } else if (scaledHeight > containerHeight) {
      scaledHeight = containerHeight;
      scaledWidth = containerHeight * svgWidthHeightRatio.value;
    }

    // Update the viewBox and dimensions
    props.svgElement.setAttribute("width", `${scaledWidth}`);
    props.svgElement.setAttribute("height", `${scaledHeight}`);
    savedSvgInContainer.value = serializer.serializeToString(props.svgElement);
  }
};

// Method to process the svg drawing and discretize it into points using the point distance set by the user
const processDrawing = () => {
  if (!savedSvg.value) {
    console.log("No SVG saved");
    return;
  }
  const elements = getElementsFromSvgString(savedSvg.value);
  let newPointsPerElement = <
    { closed: boolean; points: { x: number; y: number }[] }[]
  >[];
  elements.forEach((element) => {
    const isClosed = ["rect", "ellipse"].includes(element.tagName);
    const points = getPointsFromElement(element, Number(pointDistance.value));
    newPointsPerElement.push({ closed: isClosed, points });
  });
  pointsPerElement.value = newPointsPerElement;
};

// Method to create a sequence of commands to draw the SVG elements on the map
const createSequenceOfCommands = (
  pointsPerElement: { closed: boolean; points: { x: number; y: number }[] }[]
) => {
  let sequenceOfCommands: Command[] = [];
  const allPoints = pointsPerElement.map((element) => element.points).flat();
  if (allPoints.length === 0) {
    console.error("No points to draw");
    return sequenceOfCommands;
  }
  // Find the global min and max points across all elements
  const globalMinX = Math.min(...allPoints.map((p) => p.x));
  const globalMaxX = Math.max(...allPoints.map((p) => p.x));
  const globalMinY = Math.min(...allPoints.map((p) => p.y));
  const globalMaxY = Math.max(...allPoints.map((p) => p.y));
  // Calculate the global width and height
  const globalWidth = globalMaxX - globalMinX;
  const globalHeight = globalMaxY - globalMinY;

  pointsPerElement.forEach((element) => {
    // Normalize the element points to the dimensions of the svg drawing
    const normalizedPoints = normalizeElementPointsToCanvas(
      element.points,
      svgWidth.value || 0,
      svgHeight.value || 0,
      globalMinX,
      globalMinY,
      globalWidth,
      globalHeight
    );
    // Localize the points on the map base on the top and left of the svg drawing
    const localizedPoints = normalizedPoints.map((point) => {
      return {
        x: point.x + (svgLeft.value || 0),
        y: point.y + (svgTop.value || 0),
      };
    });
    // Convert the pixel points to map coordinates points
    const mapCoordinates = localizedPoints.map((point) =>
      canvasToMapCoordinates(
        point.x,
        point.y,
        mapObject.value,
        scaleFactor.value
      )
    );
    // Algorithm to draw the element on the map
    // First command put the pen up and move to the first point oriented to the second point
    sequenceOfCommands.push({
      type: "pen_state",
      data: { pen_state: "up" },
    });
    sequenceOfCommands.push({
      type: "move",
      data: { pose: fromArrowToPose(mapCoordinates[0], mapCoordinates[1]) },
    });
    // Put the pen down
    sequenceOfCommands.push({
      type: "pen_state",
      data: { pen_state: "down" },
    });

    for (let i = 1; i <= mapCoordinates.length - 2; i++) {
      // Reach the next point by keeping the same orientation
      sequenceOfCommands.push({
        type: "move",
        data: {
          pose: fromArrowToPose(
            mapCoordinates[i - 1],
            mapCoordinates[i],
            "end"
          ),
        },
      });
      // Once reached the point, move the pen up
      sequenceOfCommands.push({
        type: "pen_state",
        data: { pen_state: "up" },
      });
      // Orient in the direction of the next point
      sequenceOfCommands.push({
        type: "move",
        data: {
          pose: fromArrowToPose(mapCoordinates[i], mapCoordinates[i + 1]),
        },
      });
      // Put the pen down
      sequenceOfCommands.push({
        type: "pen_state",
        data: { pen_state: "down" },
      });
    }

    // Move to the last point
    sequenceOfCommands.push({
      type: "move",
      data: {
        pose: fromArrowToPose(
          mapCoordinates[mapCoordinates.length - 2],
          mapCoordinates[mapCoordinates.length - 1],
          "end"
        ),
      },
    });

    // Put the pen up
    sequenceOfCommands.push({
      type: "pen_state",
      data: { pen_state: "up" },
    });
  });
  return sequenceOfCommands;
};

// Method to handle the draw it button, emits the sequence of commands to draw the SVG elements on the map
const handleDrawIt = () => {
  const sequenceOfCommands = createSequenceOfCommands(pointsPerElement.value);
  emit("sequenceOfCommands", sequenceOfCommands);
};
</script>
<style>
.no-border {
  border: none;
}

.buttons-container {
  display: flex;
  justify-content: center;
  margin-top: 1rem;
  flex-direction: row;
}

.slider-container,
.drawing-dim-container {
  margin-top: 1rem;
}

.point-distance-input {
  position: relative;
  top: -1px;
  width: 3rem;
}

.preview-container {
  display: flex;
  flex-direction: row;
  justify-content: center;
  align-items: flex-start;
  width: 100%;
}

.preview-right,
.preview-left {
  margin: 1rem;
  border: 1px solid #ccc;
  padding: 1rem;
  display: flex;
  justify-content: center;
  align-items: center;
}

.svg-container {
  max-height: 100%; /* Ensure it doesn’t overflow vertically */
  display: flex;
  justify-content: center;
  align-items: center;
}
</style>
