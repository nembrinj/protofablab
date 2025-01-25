<template>
  <div>
    <div style="position: relative; display: inline-block">
      <canvas
        ref="mapCanvas"
        :width="canvasWidth"
        :height="canvasHeight"
        @mousedown="startArrow"
        @mousemove="generateArrow"
        @mouseup="endArrow"
      ></canvas>
      <!-- Resize handle -->
      <div
        v-if="!disableResizeMap"
        ref="resizeHandle"
        @mousedown="startResize"
        style="
          position: absolute;
          bottom: -10px;
          right: -15px;
          width: 15px;
          height: 15px;
          background: #00b7ff;
          cursor: nwse-resize;
        "
      ></div>
      <div
        ref="draggableSvg"
        v-if="!disableSvgVisualization"
        v-html="drawingSvgContent"
        style="position: absolute; top: 0; left: 0"
        :class="{ draggable: !disableSvgDrag, 'non-draggable': disableSvgDrag }"
        @mousedown="startDragSvg"
      ></div>
    </div>
  </div>
</template>

<script setup lang="ts">
/**
 * MapVisualizerComponent.vue
 * It allows to visualize a map, resize it, draw an arrow to generate a goal pose, visualize an svg,
 * drag the svg around the map, plot a set of points on the map and plot the robot position on the map.
 */
import { ref, onMounted, watch, onUnmounted } from "vue";
import { MapObject } from "@/types/map";
import { Pose } from "@/types/geometry";
import {
  canvasToMapCoordinates,
  mapCoordinatesToCanvas,
  fromArrowToPose,
} from "@/composables/processDrawing";

// Define the props of the component
const props = defineProps<{
  mapData: MapObject | null;
  scaleFactor?: number;
  disableSendGoal?: boolean;
  disableResizeMap?: boolean;
  drawingSvg?: string;
  drawingSvgWidth?: number;
  drawingSvgHeight?: number;
  drawingSvgRatio?: number;
  drawingSvgTop?: number;
  drawingSvgLeft?: number;
  disableSvgVisualization?: boolean;
  disableSvgDrag?: boolean;
  robotPosition?: { x: number; y: number } | null;
  monitoredData?: { x: number; y: number }[];
}>();

// Define the emits of the component
const emit = defineEmits<{
  (event: "goal", goal: Pose | null): void;
  (event: "canvasSize", size: { width: number; height: number }): void;
  (
    event: "svgInfo",
    info: { width: number; height: number; top: number; left: number }
  ): void;
  (event: "newScaleFactor", scaleFactor: number): void;
}>();

// Ref to the canvas element
const mapCanvas = ref<HTMLCanvasElement | null>(null);
// Canvas dimensions
const canvasWidth = ref<number | undefined>();
const canvasHeight = ref<number | undefined>();
// Map coming from the backend
const mapData = ref<number[] | null>(null);
// Map as an image data to draw on the canvas
const mapImageData = ref<ImageData | null>(null);
// Canvas context
const ctx = ref<CanvasRenderingContext2D | null>(null);
// Variables to handle map resizing
const scaleFactor = ref(props.scaleFactor || 2);
const isResizing = ref(false);
const resizeStart = ref({ x: 0, y: 0 });

// Variables for arrow drawing
const arrowStart = ref<{ x: number; y: number } | null>(null); // { x, y } starting point of the arrow
const arrowEnd = ref<{ x: number; y: number } | null>(null); // { x, y } end point of the arrow (dragging)
const isDraggingArrow = ref(false);

// Variables for svg drawing
const draggableSvg = ref<HTMLElement | null>(null);
const drawingSvgContent = ref<string | null | undefined>(
  props.drawingSvg || null
);

// When the component is mounted the canvas context is set and on it the map and the svg are drawn
onMounted(() => {
  if (!mapCanvas.value) return;
  ctx.value = mapCanvas.value?.getContext("2d");
  if (props.mapData) {
    console.log("Map data is available");
    const { data, width, height } = scaleOccupancyGrid(
      props.mapData.map.data,
      props.mapData.map.info.width,
      props.mapData.map.info.height,
      scaleFactor.value
    );
    canvasWidth.value = width;
    canvasHeight.value = height;
    emit("canvasSize", { width, height });
    mapData.value = data;
    // Remove cached image data
    mapImageData.value = null;
    drawMap();
  }
  if (props.drawingSvg) {
    drawingSvgContent.value = props.drawingSvg;
    if (draggableSvg.value) {
      draggableSvg.value.style.top = `${props.drawingSvgTop}px` || "0px";
      draggableSvg.value.style.left = `${props.drawingSvgLeft}px` || "0px";
      // Set the initial size of the SVG based on the drawingSvgWidth and drawingSvgRatio
      if (props.drawingSvgWidth && props.drawingSvgRatio) {
        draggableSvg.value.style.width = `${props.drawingSvgWidth}px`;
        draggableSvg.value.style.height = `${
          props.drawingSvgWidth / props.drawingSvgRatio
        }px`;
      } else if (props.drawingSvgRatio) {
        if (props.drawingSvgRatio > 1) {
          draggableSvg.value.style.width = `100px`;
          draggableSvg.value.style.height = `${100 / props.drawingSvgRatio}px`;
        } else {
          draggableSvg.value.style.height = `100px`;
          draggableSvg.value.style.width = `${100 * props.drawingSvgRatio}px`;
        }
      }
      emit("svgInfo", {
        width: parseInt(draggableSvg.value.style.width),
        height: parseInt(draggableSvg.value.style.height),
        top: parseInt(draggableSvg.value.style.top),
        left: parseInt(draggableSvg.value.style.left),
      });
    }
  }
});

onUnmounted(() => {
  window.removeEventListener("mousemove", resize);
  window.removeEventListener("mouseup", endResize);
});

// Watch for changes in the map data and in case the map is changed the canvas is redrawn
// and the mapImageData is recreated to be drawn on the canvas.
watch(
  () => props.mapData,
  (newVal) => {
    if (newVal) {
      const { data, width, height } = scaleOccupancyGrid(
        newVal.map.data,
        newVal.map.info.width,
        newVal.map.info.height,
        scaleFactor.value
      );

      canvasWidth.value = width;
      canvasHeight.value = height;
      emit("canvasSize", { width, height });
      mapData.value = data;
      mapImageData.value = null;
      drawMap();
    }
  }
);

// Watch for changes in the monitored data and in case the monitored data is changed the canvas is redrawn,
// monitored data is used to plot points on the map where the robot has been with the pen down.
watch(
  () => props.monitoredData,
  () => {
    drawMap();
    if (props.monitoredData) {
      drawPoints(props.monitoredData);
    }
  },
  { deep: true }
);

// Watch for changes in the robot position and in case the robot position is changed the canvas is redrawn.
watch(
  () => props.robotPosition,
  () => {
    drawMap();
    drawPoints(props.monitoredData || []);
    if (props.robotPosition) {
      drawRobot(props.robotPosition);
    }
  },
  { deep: true }
);

// Watch for changes in the scale factor and in case the scale factor is changed this changes is emitted to the
// parent component.
watch(scaleFactor, () => {
  emit("newScaleFactor", scaleFactor.value);
});

// Method to draw the map on the canvas
const drawMap = async () => {
  if (
    !canvasHeight.value ||
    !canvasWidth.value ||
    !mapData.value ||
    !ctx.value
  ) {
    console.warn("Canvas or map data is not ready.");
    return;
  }
  if (mapImageData.value) {
    ctx.value.putImageData(mapImageData.value, 0, 0);
    return;
  }
  const imageData = ctx.value.createImageData(
    canvasWidth.value,
    canvasHeight.value
  );
  // Transform map data to flip vertically
  // ROS maps start from bottom left, canvas starts from top left
  for (let row = 0; row < canvasHeight.value; row++) {
    for (let col = 0; col < canvasWidth.value; col++) {
      // Source index in the map data
      const srcIdx = row * canvasWidth.value + col;
      // Destination index after flipping
      const dstIdx =
        ((canvasHeight.value - 1 - row) * canvasWidth.value + col) * 4;
      const value = mapData.value[srcIdx];
      // Map the value to an RGBA color
      const color = mapValueToRGB(value);
      // Set the pixel color in the image data
      imageData.data[dstIdx] = color[0];
      imageData.data[dstIdx + 1] = color[1];
      imageData.data[dstIdx + 2] = color[2];
      imageData.data[dstIdx + 3] = color[3];
    }
  }
  // Simulate an asynchronous operation so that the UI can be updated
  await new Promise((resolve) => setTimeout(resolve, 0));
  mapImageData.value = imageData;
  ctx.value.putImageData(imageData, 0, 0);
};

// Draw set of points on the canvas, used to plot the points where the robot has been with the pen down
const drawPoints = (points: { x: number; y: number }[]) => {
  if (!ctx.value || !canvasWidth.value || !canvasHeight.value) return;
  points.forEach((point) => {
    const canvasPoint = mapCoordinatesToCanvas(
      point.x,
      point.y,
      props.mapData,
      scaleFactor.value
    );
    if (ctx.value) {
      ctx.value.beginPath();
      // Draw a green circle
      ctx.value.arc(canvasPoint.x, canvasPoint.y, 2, 0, Math.PI * 2);
      ctx.value.fillStyle = "green";
      ctx.value.fill();
      ctx.value.closePath();
    }
  });
};

// Method to draw the robot on the canvas
const drawRobot = (point: { x: number; y: number }) => {
  if (!ctx.value || !canvasWidth.value || !canvasHeight.value) return;
  const canvasPoint = mapCoordinatesToCanvas(
    point.x,
    point.y,
    props.mapData,
    scaleFactor.value
  );
  if (ctx.value) {
    ctx.value.beginPath();
    // Draw an orange circle
    ctx.value.arc(canvasPoint.x, canvasPoint.y, 10, 0, Math.PI * 2);
    ctx.value.fillStyle = "orange";
    ctx.value.fill();
    ctx.value.closePath();
  }
};

// Function to obtain a resized occupancy grid
const scaleOccupancyGrid = (
  grid: number[],
  originalWidth: number,
  originalHeight: number,
  scaleFactor: number
) => {
  // Calculate new dimensions
  const newWidth = Math.floor(originalWidth * scaleFactor);
  const newHeight = Math.floor(originalHeight * scaleFactor);

  // Ensure dimensions are valid
  if (newWidth <= 0 || newHeight <= 0) {
    throw new Error(`Invalid grid dimensions: ${newWidth}x${newHeight}`);
  }

  const newGrid = new Array(newWidth * newHeight).fill(-1);

  // Populate the new grid
  for (let row = 0; row < newHeight; row++) {
    for (let col = 0; col < newWidth; col++) {
      const originalRow = Math.floor(row / scaleFactor);
      const originalCol = Math.floor(col / scaleFactor);

      const originalIndex = originalRow * originalWidth + originalCol;

      // Ensure the original index is valid
      if (
        originalRow >= originalHeight ||
        originalCol >= originalWidth ||
        originalIndex >= grid.length
      ) {
        continue;
      }

      newGrid[row * newWidth + col] = grid[originalIndex];
    }
  }

  return { data: newGrid, width: newWidth, height: newHeight };
};

// Function to map a value of the occupancy grid to an RGBA color
const mapValueToRGB = (value: number) => {
  if (value === -1) return [128, 128, 128, 255]; // Gray
  if (value === 0) return [255, 255, 255, 255]; // White
  if (value === 100) return [0, 0, 0, 255]; // Black
  return [255, 255, 255, 255]; // Default -> White
};

/***************************************************************************/
/**************************Methods to resize the map**********************/
const startResize = (event: MouseEvent) => {
  isResizing.value = true;
  resizeStart.value = { x: event.clientX, y: event.clientY };
  window.addEventListener("mousemove", resize);
  window.addEventListener("mouseup", endResize);
  // Set the drawing position to the top-left corner (avoid possible svg overflow)
  if (draggableSvg.value) {
    draggableSvg.value.style.top = "0px";
    draggableSvg.value.style.left = "0px";
  }
};

const resize = (event: MouseEvent) => {
  if (!isResizing.value || !props.mapData) return;

  try {
    const dx = event.clientX - resizeStart.value.x;
    let newScaleFactor = scaleFactor.value + dx / 10000;

    // Constrain scale factor to a valid range
    newScaleFactor = Math.max(0.1, Math.min(newScaleFactor, 10));
    scaleFactor.value = newScaleFactor;

    // Calculate new dimensions
    const newWidth = Math.max(
      1,
      Math.floor(props.mapData.map.info.width * newScaleFactor)
    );
    const newHeight = Math.max(
      1,
      Math.floor(props.mapData.map.info.height * newScaleFactor)
    );

    // Update canvas dimensions
    canvasWidth.value = newWidth;
    canvasHeight.value = newHeight;
    emit("canvasSize", { width: newWidth, height: newHeight });

    // Validate and create a placeholder array
    const placeholderGrid = new Array(newWidth * newHeight).fill(-1);
    mapData.value = placeholderGrid;
  } catch (error) {
    console.error("Error during resizing:", error);
  }
};

const endResize = () => {
  if (!props.mapData) return;
  isResizing.value = false;
  const { data, width, height } = scaleOccupancyGrid(
    props.mapData.map.data,
    props.mapData.map.info.width,
    props.mapData.map.info.height,
    scaleFactor.value
  );

  canvasWidth.value = width;
  canvasHeight.value = height;
  mapData.value = data;
  // Remove cached image data
  mapImageData.value = null;
  // Redraw map on the canvas
  drawMap();
  // Reset goal
  emit("goal", null);
  window.removeEventListener("mousemove", resize);
  window.removeEventListener("mouseup", endResize);
};
/***************************************************************************/
/*********************Methods to draw the arrow pose**********************/
const startArrow = (event: MouseEvent) => {
  if (props.disableSendGoal || !mapCanvas.value) return;
  const rect = mapCanvas.value.getBoundingClientRect();
  arrowStart.value = {
    x: event.clientX - rect.left,
    y: event.clientY - rect.top,
  };
  // reset the goal
  emit("goal", null);
  isDraggingArrow.value = true;
};

const generateArrow = (event: MouseEvent) => {
  if (isDraggingArrow.value && arrowStart.value && mapCanvas.value) {
    const rect = mapCanvas.value.getBoundingClientRect();
    arrowEnd.value = {
      x: event.clientX - rect.left,
      y: event.clientY - rect.top,
    };
    // Calculate max arrow length based on scale factor
    const maxArrowLength = 40 * scaleFactor.value; // Adjust based on preference
    const dx = arrowEnd.value.x - arrowStart.value.x;
    const dy = arrowEnd.value.y - arrowStart.value.y;

    // Limit the length of the arrow based on maxArrowLength
    const arrowLength = Math.min(Math.sqrt(dx * dx + dy * dy), maxArrowLength);

    // Normalize direction vector (dx, dy)
    const angle = Math.atan2(dy, dx);
    arrowEnd.value = {
      x: arrowStart.value.x + arrowLength * Math.cos(angle),
      y: arrowStart.value.y + arrowLength * Math.sin(angle),
    };

    drawMap();
    drawArrow();
  }
};

const endArrow = () => {
  if (isDraggingArrow.value && arrowStart.value && arrowEnd.value) {
    isDraggingArrow.value = false;

    // Convert arrow start and end points to map coordinates
    const arrowStartMap = canvasToMapCoordinates(
      arrowStart.value.x,
      arrowStart.value.y,
      props.mapData,
      scaleFactor.value
    );

    const arrowEndMap = canvasToMapCoordinates(
      arrowEnd.value.x,
      arrowEnd.value.y,
      props.mapData,
      scaleFactor.value
    );

    // Convert arrow start and end points to a pose
    const pose = fromArrowToPose(arrowStartMap, arrowEndMap);

    emit("goal", pose);
  }
};

const drawArrow = () => {
  if (!arrowStart.value || !arrowEnd.value || !ctx.value) return;

  const ctxLocal = ctx.value;

  // Draw the new arrow (this is the actual drawing part)
  ctxLocal.beginPath();
  ctxLocal.moveTo(arrowStart.value.x, arrowStart.value.y);
  ctxLocal.lineTo(arrowEnd.value.x, arrowEnd.value.y);

  // Calculate orientation
  const dx = arrowEnd.value.x - arrowStart.value.x;
  const dy = arrowEnd.value.y - arrowStart.value.y;
  // Orientation in radians
  const angle = Math.atan2(dy, dx);
  // Draw arrowhead
  const arrowHeadLength = 10; // Length of the arrowhead
  ctxLocal.lineTo(
    arrowEnd.value.x - arrowHeadLength * Math.cos(angle - Math.PI / 6),
    arrowEnd.value.y - arrowHeadLength * Math.sin(angle - Math.PI / 6)
  );
  ctxLocal.moveTo(arrowEnd.value.x, arrowEnd.value.y);
  ctxLocal.lineTo(
    arrowEnd.value.x - arrowHeadLength * Math.cos(angle + Math.PI / 6),
    arrowEnd.value.y - arrowHeadLength * Math.sin(angle + Math.PI / 6)
  );

  // Apply stroke style
  ctxLocal.strokeStyle = "red";
  ctxLocal.lineWidth = 2;
  ctxLocal.stroke();
};
/***************************************************************************/
/**********************SVG Visualization************************************/
// Watch for changes in the drawingSvg and in case the drawingSvg is changed the svg is updated
watch(
  () => props.drawingSvg,
  (newVal) => {
    drawingSvgContent.value = newVal;
    if (draggableSvg.value) {
      draggableSvg.value.style.top = `${props.drawingSvgTop}px` || "0px";
      draggableSvg.value.style.left = `${props.drawingSvgLeft}px` || "0px";
    }
    // Set the size of the SVG based on the drawingSvgWidth and drawingSvgRatio
    if (draggableSvg.value && props.drawingSvgWidth && props.drawingSvgRatio) {
      draggableSvg.value.style.width = `${props.drawingSvgWidth}px`;
      draggableSvg.value.style.height = `${
        props.drawingSvgWidth / props.drawingSvgRatio
      }px`;
    } else if (draggableSvg.value && props.drawingSvgRatio) {
      if (props.drawingSvgRatio > 1) {
        draggableSvg.value.style.width = `100px`;
        draggableSvg.value.style.height = `${100 / props.drawingSvgRatio}px`;
      } else {
        draggableSvg.value.style.height = `100px`;
        draggableSvg.value.style.width = `${100 * props.drawingSvgRatio}px`;
      }
      emit("svgInfo", {
        width: parseInt(draggableSvg.value.style.width),
        height: parseInt(draggableSvg.value.style.height),
        top: parseInt(draggableSvg.value.style.top),
        left: parseInt(draggableSvg.value.style.left),
      });
    }
  }
);

// Watch for changes in the drawingSvgWidth and in case the drawingSvgWidth is changed the svg is updated
watch(
  () => props.drawingSvgWidth,
  (newVal) => {
    if (draggableSvg.value) {
      draggableSvg.value.style.top = `${props.drawingSvgTop}px` || "0px";
      draggableSvg.value.style.left = `${props.drawingSvgLeft}px` || "0px";
    }
    if (draggableSvg.value && newVal && props.drawingSvgRatio) {
      updateSvgSize({ id: "width", value: newVal }, draggableSvg.value);
    }
  }
);

// Watch for changes in the drawingSvgRatio and in case the drawingSvgRatio is changed the svg is updated
watch(
  () => props.drawingSvgRatio,
  (newVal) => {
    if (draggableSvg.value) {
      draggableSvg.value.style.top = `${props.drawingSvgTop}px` || "0px";
      draggableSvg.value.style.left = `${props.drawingSvgLeft}px` || "0px";
    }
    if (draggableSvg.value && newVal) {
      if (!props.drawingSvgWidth) {
        return;
      } else {
        updateSvgSize(
          { id: "width", value: props.drawingSvgWidth },
          draggableSvg.value
        );
      }
    }
  }
);

// Watch for changes in the drawingSvgTop and in case the drawingSvgTop is changed the svg is updated
watch(
  () => props.drawingSvgTop,
  (newVal) => {
    if (draggableSvg.value) {
      draggableSvg.value.style.top = `${newVal}px` || "0px";
    }
  }
);

// Watch for changes in the drawingSvgLeft and in case the drawingSvgLeft is changed the svg is updated
watch(
  () => props.drawingSvgLeft,
  (newVal) => {
    if (draggableSvg.value) {
      draggableSvg.value.style.left = `${newVal}px` || "0px";
    }
  }
);

// Method to update the size of the SVG
const updateSvgSize = (
  newValue: { id: string; value: number },
  svgElement: HTMLElement
) => {
  if (!svgElement) return;
  if (newValue.id == "width" && props.drawingSvgRatio) {
    svgElement.style.width = `${newValue.value}px`;
    svgElement.style.height = `${newValue.value / props.drawingSvgRatio}px`;
  } else if (newValue.id == "height" && props.drawingSvgRatio) {
    svgElement.style.height = `${newValue.value}px`;
    svgElement.style.width = `${props.drawingSvgRatio * newValue.value}px`;
  } else {
    console.log("Invalid value for updateSvgSize");
  }
  emit("svgInfo", {
    width: parseInt(svgElement.style.width),
    height: parseInt(svgElement.style.height),
    top: parseInt(svgElement.style.top),
    left: parseInt(svgElement.style.left),
  });
};

// Variables to handle the dragging of the SVG
let isDraggingSvg = false;
let startXSvg = 0;
let startYSvg = 0;

// Method to start the drag of the SVG
const startDragSvg = (event: MouseEvent) => {
  if (props.disableSvgVisualization || props.disableSvgDrag) return;
  isDraggingSvg = true;
  startXSvg =
    event.clientX - (draggableSvg.value?.getBoundingClientRect().left || 0);
  startYSvg =
    event.clientY - (draggableSvg.value?.getBoundingClientRect().top || 0);
  document.addEventListener("mousemove", handleDragSvg);
  document.addEventListener("mouseup", stopDragSvg);
};

// Method to handle the drag of the SVG
const handleDragSvg = (event: MouseEvent) => {
  if (!isDraggingSvg || !draggableSvg.value || !mapCanvas.value) return;

  // Calculate the new position
  const containerRect = mapCanvas.value.getBoundingClientRect();
  let newLeft = event.clientX - startXSvg;
  let newTop = event.clientY - startYSvg;

  // Constrain within the mapCanvas boundaries
  newLeft = Math.max(
    containerRect.left,
    Math.min(newLeft, containerRect.right - draggableSvg.value.clientWidth)
  );
  newTop = Math.max(
    containerRect.top,
    Math.min(newTop, containerRect.bottom - draggableSvg.value.clientHeight)
  );

  // Update the SVG position
  draggableSvg.value.style.left = `${newLeft - containerRect.left}px`;
  draggableSvg.value.style.top = `${newTop - containerRect.top}px`;

  emit("svgInfo", {
    width: parseInt(draggableSvg.value.style.width),
    height: parseInt(draggableSvg.value.style.height),
    top: parseInt(draggableSvg.value.style.top),
    left: parseInt(draggableSvg.value.style.left),
  });
};

// Method to stop the drag of the SVG
const stopDragSvg = () => {
  isDraggingSvg = false;
  document.removeEventListener("mousemove", handleDragSvg);
  document.removeEventListener("mouseup", stopDragSvg);
};
</script>

<style>
canvas {
  border: 1px solid black;
}
div[ref="resizeHandle"] {
  background-color: gray;
  border-radius: 50%;
}

div[ref="resizeHandleSvg"] {
  background-color: gray;
  border-radius: 50%;
}

.draggable {
  cursor: grab;
}

.non-draggable {
  cursor: default;
}
</style>
