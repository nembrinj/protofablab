import api from "./api";
import { Pose, Command } from "../types/geometry";
import { MapObject, MonitoringData } from "@/types/map";

// Send to the backend a request to change the pen state
export const changePenState = (penState: string) => {
  return api.post<{ success: boolean }>("/change_pen_state", { penState });
};

// Get the 2d map
export const getMap = () => {
  return api.get<MapObject>("/get_map");
};

// Send a goal
export const sendGoal = (pose: Pose) => {
  return api.post<{ success: boolean }>("/send_goal", { pose });
};

// Send a sequence of commands
export const sendCommands = (commands: Command[]) => {
  return api.post<{ success: boolean }>("/sequence_of_commands", { commands });
};

// Get monitoring data
export const getMonitoringData = () => {
  return api.get<MonitoringData[]>("/get_monitoring_data");
};

// Stop the process
export const stopProcess = () => {
  return api.post<{ success: boolean }>("/stop_process");
};
