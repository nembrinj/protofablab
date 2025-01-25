<template>
  <div class="buttons-container">
    <button class="action-button" @click="handleChangePenState('up')">
      PEN UP
    </button>
    <button class="action-button" @click="handleChangePenState('down')">
      PEN DOWN
    </button>
    <button class="action-button" @click="handleGetMap">GET MAP</button>
    <button class="action-button" @click="handleSubmitGoal" v-if="currentPose">
      SEND GOAL
    </button>
  </div>
  <MapVisualizerComponent
    v-if="mapObject"
    :mapData="mapObject"
    :disableSvgVisualization="true"
    @goal="handleGoal"
  />
</template>

<script setup lang="ts">
/**
 * SendGoalTestComponent.vue
 * It allows to send a goal or a pen state to the backend and visualize the map.
 */
import { changePenState, getMap, sendGoal } from "@/api/userService";
import { Pose } from "@/types/geometry";
import { MapObject } from "@/types/map";
import { onMounted, ref } from "vue";
import MapVisualizerComponent from "./MapVisualizerComponent.vue";

// Variable that stores the map
const mapObject = ref<MapObject | null>(null);
// Variable that stores the current pose to send a goal
const currentPose = ref<Pose | null>(null);

// Method to send the command to change the pen state to the backend
const handleChangePenState = async (state: string) => {
  await changePenState(state);
};

// Method to fetch the map
const handleGetMap = async () => {
  const response = await getMap();
  if (response.status === 200) {
    mapObject.value = response.data;
  } else {
    console.error("Error getting map data");
  }
};

// Fetch the map when the component is mounted
onMounted(() => {
  handleGetMap();
});

// Method to handle a goal coming from the MapVisualizerComponent
const handleGoal = (pose: Pose | null) => {
  currentPose.value = pose;
};

// Method to send the goal to the backend
const handleSubmitGoal = async () => {
  if (currentPose.value === null) {
    return;
  }
  await sendGoal(currentPose.value);
};
</script>
<style>
.buttons-container {
  display: flex;
  justify-content: center;
  margin-top: 1rem;
  flex-direction: row;
}
</style>
