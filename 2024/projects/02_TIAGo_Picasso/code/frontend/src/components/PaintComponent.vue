<template>
  <vp-editor
    v-model:history="history"
    v-model:settings="settings"
    @save="handleSave"
    :tools="tools"
  >
  </vp-editor>
</template>

<script setup lang="ts">
/**
 * PaintComponent.vue
 * It allows to draw on a canvas and save the SVG as a string
 */
import { ref } from "vue";
import "vue-paint/themes/default.css";
import {
  VpEditor,
  useFreehand,
  useLine,
  useRectangle,
  useEraser,
  useEllipse,
  useMove,
  createSettings,
} from "vue-paint";
import type { ImageHistory } from "vue-paint";

const emit = defineEmits<{
  (event: "svg", svg: string): void;
}>();

// Define the tools that are usable in the editor
const tools = [
  useFreehand(),
  useLine(),
  useRectangle(),
  useEllipse(),
  useEraser(),
  useMove(),
];

// Initialize history to keep track of the drawing history
const history = ref<ImageHistory<typeof tools>>([]);

// Initialize settings with starting color and tool options
const settings = createSettings(tools, { color: "#000000" });

// Handle the save event to emit the SVG string
const handleSave = (arg: any) => {
  const serializer = new XMLSerializer();
  const svgString = serializer.serializeToString(arg.svg);
  const cleanedSvgString = svgString.replace(/svg:style/g, "style");
  emit("svg", cleanedSvgString);
};
</script>

<style>
.vp-image {
  border: 1px solid #000;
}
</style>
