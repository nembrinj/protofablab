<template>
  <canvas ref="canvas" :width="canvasWidth" :height="canvasHeight"></canvas>
</template>

<script setup lang="ts">
/**
 * SetOfPointsVisualizerComponent.vue
 * This component is responsible for rendering a set of points on a canvas
 */
import { ref, onMounted, watch } from "vue";
import { normalizePointsToCanvas } from "../composables/processDrawing";

interface Props {
  points: { x: number; y: number }[];
  canvasWidth: number;
  canvasHeight: number;
}

// Define props
const props = defineProps<Props>();
// Ref to the canvas element
const canvas = ref<HTMLCanvasElement | null>(null);
// Set of points scaled to the canvas size
const scaledSetOfPoints = ref<{ x: number; y: number }[]>([]);

// Render points when the component is mounted
onMounted(() => {
  if (props) {
    drawPoints();
  }
});

// Watch for changes in the points prop and re-render
watch(
  () => props.points,
  () => {
    drawPoints();
  },
  { deep: true }
);

watch(
  () => props.canvasWidth,
  () => {
    drawPoints();
  }
);

watch(
  () => props.canvasHeight,
  () => {
    drawPoints();
  }
);

// Method to draw points on the canvas
const drawPoints = () => {
  if (props) {
    scalePoints();
    renderPoints();
  }
};

// Method to scale points to the canvas size
const scalePoints = () => {
  scaledSetOfPoints.value = normalizePointsToCanvas(
    props.points,
    props.canvasWidth,
    props.canvasHeight
  );
};

// Method to render points on the canvas
const renderPoints = () => {
  const ctx = canvas.value?.getContext("2d");
  if (!ctx) {
    console.error("Canvas context not available");
    return;
  }

  ctx.clearRect(0, 0, props.canvasWidth, props.canvasHeight);

  ctx.fillStyle = "black";
  for (const point of scaledSetOfPoints.value) {
    ctx.beginPath();
    ctx.arc(point.x, point.y, 2, 0, Math.PI * 2);
    ctx.fill();
  }
};
</script>

<style></style>
