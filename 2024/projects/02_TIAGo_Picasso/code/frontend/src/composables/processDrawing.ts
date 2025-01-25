import { MapObject } from "@/types/map";

// Function to split the path data into subpaths at each 'M' command
function splitPathIntoSubPaths(pathData: string): string[] {
  // Remove leading whitespace and 'M' from the start if present
  const pathWithoutLeadingM = pathData.trim().replace(/^M/, "");

  // Split the path on each 'M'
  const subpaths = pathWithoutLeadingM
    .split("M")
    .map((subpath) => {
      return "M" + subpath.trim();
    })
    // remove empty strings
    .filter(Boolean);
  return [...subpaths];
}

// Function to get SVG elements from an SVG string
export function getElementsFromSvgString(
  svgString: string
): SVGGeometryElement[] {
  const parser = new DOMParser();
  const doc = parser.parseFromString(svgString, "image/svg+xml");

  // Check for parsing errors
  const errorNode = doc.querySelector("parsererror");
  if (errorNode) {
    console.error("Error parsing SVG:", errorNode.textContent);
    return [];
  }

  // Query all supported geometric elements
  const elements = Array.from(
    doc.querySelectorAll("path, rect, ellipse, line")
  ) as SVGGeometryElement[];

  // Remove the elements of the svg that are not part of the drawing but are there for the
  // resize handles
  const filteredElements = elements.filter(
    (element) => !element.id || !element.id.includes("handle")
  );

  const resultElements: SVGGeometryElement[] = [];

  filteredElements.forEach((element) => {
    if (element instanceof SVGPathElement) {
      // Get the path's data (d attribute)
      const pathData = element.getAttribute("d") || "";

      // Split the path into subpaths based on the 'M' command
      const subPaths = splitPathIntoSubPaths(pathData);

      // For each subpath, clone the original element and set its d attribute
      subPaths.forEach((subPath) => {
        const newPath = element.cloneNode() as SVGPathElement;
        newPath.setAttribute("d", subPath); // Set the subpath

        // Add the cloned path with the updated d attribute to the result
        resultElements.push(newPath);
      });
    } else {
      // For non-path elements, just add them as they are
      resultElements.push(element);
    }
  });

  return resultElements;
}

// Function to convert relative commands to absolute
function convertToAbsoluteCommands(pathData: string): string {
  // Split the path into segments based on commands
  const commands = pathData.match(/[a-df-zA-DF-Z][^a-df-zA-DF-Z]*/g) || [];
  let resultPath = "";

  let currentX = 0;
  let currentY = 0;

  commands.forEach((segment) => {
    let command = segment[0];
    let params = segment.slice(1).trim();
    let numbers = params.match(/-?\d*\.?\d+/g) || [];

    if (command === command.toUpperCase()) {
      // If the command is already uppercase (absolute), just append it
      resultPath += segment;
      if ((command === "M" || command === "L") && numbers.length === 2) {
        currentX = parseFloat(numbers[0]);
        currentY = parseFloat(numbers[1]);
      }
    } else if (numbers.length === 2) {
      // Convert relative commands to absolute
      switch (command) {
        case "m": // move to (dx, dy)
          currentX += parseFloat(numbers[0]);
          currentY += parseFloat(numbers[1]);
          resultPath += `M${currentX},${currentY}`;
          break;
        case "l": // line to (dx, dy)
          currentX += parseFloat(numbers[0]);
          currentY += parseFloat(numbers[1]);
          resultPath += `L${currentX},${currentY}`;
          break;
        case "h": // horizontal line to (dx)
          currentX += parseFloat(numbers[0]);
          resultPath += `H${currentX}`;
          break;
        case "v": // vertical line to (dy)
          currentY += parseFloat(numbers[0]);
          resultPath += `V${currentY}`;
          break;
        case "z": // close path
          resultPath += "Z";
          break;
        default:
          resultPath += segment; // Append as is for unknown commands
      }
    }
  });
  return resultPath;
}

// Function to wrap SVG elements in an SVG root element
export function wrapSvgElements(elements: SVGGeometryElement[]) {
  const svgNamespace = "http://www.w3.org/2000/svg";

  // Create a new SVG root element
  const svgRoot = document.createElementNS(svgNamespace, "svg");

  // Set default attributes for the SVG root to ensure proper rendering
  svgRoot.setAttribute("xmlns", svgNamespace);
  svgRoot.setAttribute("viewBox", "0 0 300 300");
  svgRoot.setAttribute("class", "no-border");

  // Append each SVG element to the new root
  elements.forEach((element) => {
    const clonedElement = element.cloneNode(true) as SVGGeometryElement;

    // If it's a path, convert its commands to absolute
    if (clonedElement.tagName === "path") {
      const pathData =
        (clonedElement as SVGPathElement).getAttribute("d") || "";
      const absolutePathData = convertToAbsoluteCommands(pathData);
      (clonedElement as SVGPathElement).setAttribute("d", absolutePathData);
    }

    // Normalize the element by removing inline styles and applying uniform styles
    clonedElement.removeAttribute("style");
    clonedElement.setAttribute("stroke", "black");
    clonedElement.setAttribute("stroke-width", "5px");
    clonedElement.setAttribute("fill", "none");

    // Ensure each element has a relevant class
    if (clonedElement.tagName === "path") {
      clonedElement.classList.add("freehand");
    } else if (clonedElement.tagName === "rect") {
      clonedElement.classList.add("rectangle");
    } else if (clonedElement.tagName === "ellipse") {
      clonedElement.classList.add("ellipse");
    } else if (clonedElement.tagName === "line") {
      clonedElement.classList.add("line");
    }

    svgRoot.appendChild(clonedElement);
  });

  // Add styles to the SVG root (styles taken from the svg editor)
  const styleElement = document.createElementNS(svgNamespace, "style");
  styleElement.textContent = `
    .freehand {
      stroke-linecap: round;
      stroke-linejoin: round;
      fill-opacity: 0;
      stroke: #000;
      stroke-width: 5;
    }
    .rectangle, .ellipse, .line {
      stroke-linecap: round;
      stroke-linejoin: round;
      fill-opacity: 0;
    }
    circle.handle {
      r: 0;
      stroke: #000;
      stroke-width: 2;
      stroke-opacity: 0.5;
      fill: #fff;
      fill-opacity: 0.3;
      transition: r 0.1s ease-out;
    }
  `;
  svgRoot.appendChild(styleElement);

  return svgRoot;
}

// Function to interpolate points between two endpoints
const interpolatePoints = (
  x1: number,
  y1: number,
  x2: number,
  y2: number,
  step: number
) => {
  const dx = x2 - x1;
  const dy = y2 - y1;
  const edgeLength = Math.sqrt(dx ** 2 + dy ** 2);
  const numSteps = Math.floor(edgeLength / step);
  const points: DOMPoint[] = [];
  // If the edge length is less than the step, return the two endpoints
  if (edgeLength < step) {
    return [new DOMPoint(x1, y1), new DOMPoint(x2, y2)];
  }
  for (let i = 0; i <= numSteps; i++) {
    // Interpolation factor
    const t = i / numSteps;
    const px = x1 + t * dx;
    const py = y1 + t * dy;
    points.push(new DOMPoint(px, py));
  }
  return points;
};

// Function to calculate the angle increment for arc length-based interpolation
function calculateAngleIncrement(
  a: number,
  b: number,
  currentAngle: number,
  distance: number,
  tolerance: number = 0.001
): number {
  // Initial guess for deltaTheta
  let deltaTheta = distance / Math.sqrt(a ** 2 + b ** 2);
  let arcLength = 0;

  while (true) {
    // Compute arc length for current deltaTheta
    arcLength =
      Math.sqrt(
        a ** 2 * Math.sin(currentAngle) ** 2 +
          b ** 2 * Math.cos(currentAngle) ** 2
      ) * deltaTheta;

    // Check if the arc length is close enough to the desired distance
    if (Math.abs(arcLength - distance) < tolerance) {
      break;
    }

    // Adjust deltaTheta
    deltaTheta *= distance / arcLength;
  }

  return deltaTheta;
}

// Function to get points from an SVG element at a fixed distance
// Supports paths, rectangles, ellipses, circles and lines
export function getPointsFromElement(
  element: SVGGeometryElement,
  distance: number
): DOMPoint[] {
  const points: DOMPoint[] = [];

  if (element instanceof SVGPathElement) {
    const length = element.getTotalLength();
    let currentPosition = 0;

    while (currentPosition <= length) {
      const point = element.getPointAtLength(currentPosition);
      points.push(new DOMPoint(point.x, point.y));
      currentPosition += distance;
    }

    // Ensure the final point of the path is included if it's not at an exact multiple of `distance`.
    if (currentPosition < length) {
      const finalPoint = element.getPointAtLength(length);
      points.push(new DOMPoint(finalPoint.x, finalPoint.y));
    }
  } else if (element instanceof SVGRectElement) {
    const x = element.x.baseVal.value;
    const y = element.y.baseVal.value;
    const width = element.width.baseVal.value;
    const height = element.height.baseVal.value;
    const pointsOnRect: DOMPoint[] = [];

    // Top edge
    pointsOnRect.push(...interpolatePoints(x, y, x + width, y, distance));

    // Right edge
    pointsOnRect.push(
      ...interpolatePoints(x + width, y, x + width, y + height, distance).slice(
        1
      )
    );

    // Bottom edge
    pointsOnRect.push(
      ...interpolatePoints(
        x + width,
        y + height,
        x,
        y + height,
        distance
      ).slice(1)
    );

    // Left edge
    pointsOnRect.push(
      ...interpolatePoints(x, y + height, x, y, distance).slice(1)
    );
    points.push(...pointsOnRect);
  } else if (
    element instanceof SVGEllipseElement ||
    element instanceof SVGCircleElement
  ) {
    const cx = element.cx.baseVal.value;
    const cy = element.cy.baseVal.value;

    const a =
      element instanceof SVGCircleElement
        ? element.r.baseVal.value // Radius for circles
        : element.rx.baseVal.value; // Semi-major axis for ellipses

    const b =
      element instanceof SVGCircleElement
        ? element.r.baseVal.value // Radius for circles
        : element.ry.baseVal.value; // Semi-minor axis for ellipses

    if (a <= b) {
      // Find bottom right quadrant points
      const pointsOnBottomRightQuadrant: DOMPoint[] = [];
      const pointsOnBottomQuadrants: DOMPoint[] = [];
      // Start angle in radians
      let angle = 0;
      while (angle < 0.5 * Math.PI) {
        const x = cx + a * Math.cos(angle);
        const y = cy + b * Math.sin(angle);
        pointsOnBottomRightQuadrant.push(new DOMPoint(x, y));
        const deltaTheta = calculateAngleIncrement(a, b, angle, distance);
        angle += deltaTheta;
      }
      const pointsOnBottomLeftQuadrant = pointsOnBottomRightQuadrant.map(
        (point) => new DOMPoint(cx - (point.x - cx), point.y)
      );

      pointsOnBottomQuadrants.push(...pointsOnBottomRightQuadrant);
      pointsOnBottomQuadrants.push(new DOMPoint(cx, cy + b));
      pointsOnBottomQuadrants.push(...pointsOnBottomLeftQuadrant.reverse());
      const pointsOnTopQuadrants = pointsOnBottomQuadrants.map(
        (point) => new DOMPoint(point.x, cy - (point.y - cy))
      );
      points.push(...pointsOnBottomQuadrants);
      points.push(...pointsOnTopQuadrants.reverse());
    } else {
      const pointsOnBottomLeftQuadrant: DOMPoint[] = [];
      const pointsOnLeftQuadrants: DOMPoint[] = [];
      let angle = 0.5 * Math.PI; // Start angle in radians
      while (angle < Math.PI) {
        const x = cx + a * Math.cos(angle);
        const y = cy + b * Math.sin(angle);
        pointsOnBottomLeftQuadrant.push(new DOMPoint(x, y));

        const deltaTheta = calculateAngleIncrement(a, b, angle, distance);
        angle += deltaTheta;
      }
      const pointsOnTopLeftQuadrant = pointsOnBottomLeftQuadrant.map(
        (point) => new DOMPoint(point.x, cy - (point.y - cy))
      );
      pointsOnLeftQuadrants.push(...pointsOnBottomLeftQuadrant);
      pointsOnLeftQuadrants.push(new DOMPoint(cx - a, cy));
      pointsOnLeftQuadrants.push(...pointsOnTopLeftQuadrant.reverse());
      const pointsOnRightQuadrants = pointsOnLeftQuadrants.map(
        (point) => new DOMPoint(cx + (cx - point.x), point.y)
      );
      points.push(...pointsOnLeftQuadrants);
      points.push(...pointsOnRightQuadrants.reverse());
    }
  } else if (element instanceof SVGLineElement) {
    const x1 = element.x1.baseVal.value;
    const y1 = element.y1.baseVal.value;
    const x2 = element.x2.baseVal.value;
    const y2 = element.y2.baseVal.value;

    points.push(...interpolatePoints(x1, y1, x2, y2, distance));
  } else if (
    element instanceof SVGPolygonElement ||
    element instanceof SVGPolylineElement
  ) {
    const pointsAttr = element.points;
    for (let i = 0; i < pointsAttr.numberOfItems; i++) {
      const point = pointsAttr.getItem(i);
      points.push(new DOMPoint(point.x, point.y));
    }
  }
  // Remove NaN values produced by the interpolation
  return points.filter((point) => !isNaN(point.x) && !isNaN(point.y));
}

// Function to normalize points to a canvas with a given width and height
export const normalizePointsToCanvas = (
  points: { x: number; y: number }[],
  canvasWidth: number,
  canvasHeight: number
) => {
  // find the bounding box of the points
  const minX = Math.min(...points.map((p) => p.x));
  const maxX = Math.max(...points.map((p) => p.x));
  const minY = Math.min(...points.map((p) => p.y));
  const maxY = Math.max(...points.map((p) => p.y));

  const boundingWidth = maxX - minX;
  const boundingHeight = maxY - minY;

  // calculate the scale factor
  const scaleFactor = Math.min(
    canvasWidth / boundingWidth,
    canvasHeight / boundingHeight
  );

  // center the points
  const offsetX = (canvasWidth - boundingWidth * scaleFactor) / 2;
  const offsetY = (canvasHeight - boundingHeight * scaleFactor) / 2;

  // normalize the points
  return points.map((p) => ({
    x: (p.x - minX) * scaleFactor + offsetX,
    y: (p.y - minY) * scaleFactor + offsetY,
  }));
};

// Function to normalize a subset of points to a canvas with a given width and height
// The subset of points is defined by the bounding box of all the points using global min x, y,
// global width and global height
export const normalizeElementPointsToCanvas = (
  points: { x: number; y: number }[],
  canvasWidth: number,
  canvasHeight: number,
  globalMinX: number,
  globalMinY: number,
  globalWidth: number,
  globalHeight: number
) => {
  const scaleFactor = Math.min(
    canvasWidth / globalWidth,
    canvasHeight / globalHeight
  );

  const offsetX = (canvasWidth - globalWidth * scaleFactor) / 2;
  const offsetY = (canvasHeight - globalHeight * scaleFactor) / 2;

  return points.map((p) => ({
    x: (p.x - globalMinX) * scaleFactor + offsetX,
    y: (p.y - globalMinY) * scaleFactor + offsetY,
  }));
};

// Function to convert canvas coordinates to map coordinates
export const canvasToMapCoordinates = (
  canvasX: number,
  canvasY: number,
  mapData: MapObject | null,
  scaleFactor: number
) => {
  if (!mapData) return { x: canvasX, y: canvasY };
  const mapHeight = mapData.map.info.height; // Height in pixels
  const resolution = mapData.map.info.resolution; // Meters per pixel
  const originX = mapData.map.info.origin.position.x; // Map origin x
  const originY = mapData.map.info.origin.position.y; // Map origin y

  // Adjust canvas coordinates based on the scale factor
  const scaledX = canvasX / scaleFactor;
  const scaledY = canvasY / scaleFactor;

  // Flip scaledY to match ROS map's coordinate system
  const mapPixelX = scaledX;
  const mapPixelY = mapHeight - scaledY;

  // Convert map pixel indices to ROS world coordinates
  const mapX = originX + mapPixelX * resolution;
  const mapY = originY + mapPixelY * resolution;

  return { x: mapX, y: mapY };
};

// Function to convert map coordinates to canvas coordinates
export const mapCoordinatesToCanvas = (
  mapX: number,
  mapY: number,
  mapData: MapObject | null,
  scaleFactor: number
) => {
  if (!mapData) return { x: mapX, y: mapY };

  const resolution = mapData.map.info.resolution; // Meters per pixel
  const originX = mapData.map.info.origin.position.x; // Map origin x
  const originY = mapData.map.info.origin.position.y; // Map origin y
  const mapHeight = mapData.map.info.height; // Height in pixels

  // Convert ROS world coordinates to map pixel indices
  const mapPixelX = (mapX - originX) / resolution;
  const mapPixelY = (mapY - originY) / resolution;

  // Flip mapPixelY to match canvas coordinate system
  const scaledY = mapHeight - mapPixelY;

  // Scale up to canvas coordinates based on the scale factor
  const canvasX = mapPixelX * scaleFactor;
  const canvasY = scaledY * scaleFactor;

  return { x: canvasX, y: canvasY };
};

// Function to derive the pose of an arrow from its start and end points.
// The orientation of the arrow is calculated based on the angle between the start and end points.
// The position of the arrow can be set to either the start or end point.
export const fromArrowToPose = (
  startPoint: { x: number; y: number },
  endPoint: { x: number; y: number },
  position?: string
) => {
  const dx = endPoint.x - startPoint.x;
  const dy = endPoint.y - startPoint.y;
  const angle = Math.atan2(dy, dx);

  // Calculate the orientation quaternion
  const orientation = {
    x: 0,
    y: 0,
    z: Math.sin(angle / 2),
    w: Math.cos(angle / 2),
  };

  if (position === "end") {
    return {
      position: { x: endPoint.x, y: endPoint.y, z: 0 },
      orientation,
    };
  } else {
    return {
      position: { x: startPoint.x, y: startPoint.y, z: 0 },
      orientation,
    };
  }
};
