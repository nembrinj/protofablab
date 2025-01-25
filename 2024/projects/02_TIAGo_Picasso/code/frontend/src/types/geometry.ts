interface Position {
  x: number;
  y: number;
  z: number;
}

interface Orientation {
  x: number;
  y: number;
  z: number;
  w: number;
}

export interface Pose {
  position: Position;
  orientation: Orientation;
}

export interface Command {
  type: string;
  data: { pose: Pose } | { pen_state: string };
}
