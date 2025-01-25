export interface MapObject {
  map: OccupancyGrid;
}

interface OccupancyGrid {
  data: number[];
  header: {
    frame_id: string;
    seq: number;
    stamp: {
      sec: number;
      nsec: number;
    };
  };
  info: {
    height: number;
    map_load_time: {
      sec: number;
      nsec: number;
    };
    origin: {
      orientation: {
        w: number;
        x: number;
        y: number;
        z: number;
      };
      position: {
        x: number;
        y: number;
        z: number;
      };
    };
    resolution: number;
    width: number;
  };
}

export interface MonitoringData {
  position: { x: number; y: number; z: number };
  pen_state: string;
}
