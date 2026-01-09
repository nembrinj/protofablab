from pathlib import Path
from typing import List, Tuple, Dict

import cv2
import numpy as np


class Anchor_LEDDetector:
    """Detects LED blobs by first looking for brightest spots, filtering non-circular ones and then, filtering by color."""

    # * Constants
    DEFAULT_MAX_BLOBS = 1
    DEFAULT_MIN_AREA = 100
    DEFAULT_EXPECTED_RADIUS = 30
    DEFAULT_MIN_BRIGHTNESS = 100
    DEFAULT_CIRCULARITY_THRESHOLD = 0.65
    DEFAULT_COLOR_THRESHOLD = 0.0  # Minimum color score (0-100) to accept LED

    # ! old depcrecated values
    # # Cyanish green
    # LOWER_GREEN = np.array([55, 100, 50])
    # UPPER_GREEN = np.array([95, 255, 255])

    # Green color range
    LOWER_GREEN = np.array([35, 20, 40])
    UPPER_GREEN = np.array([85, 255, 255])

    def __init__(
        self,
        max_blobs: int = DEFAULT_MAX_BLOBS,
        min_area: int = DEFAULT_MIN_AREA,
        expected_radius: int = DEFAULT_EXPECTED_RADIUS,
    ) -> None:
        """
        Initialize LED detector.

        Args:
            max_blobs: Maximum number of LED blobs to detect
            min_area: Minimum blob area
            expected_radius: More or less expected blob radius
        """
        self.max_blobs = max_blobs
        self.min_area = min_area
        self.expected_radius = expected_radius

    def detect_leds_with_adaptive_radius(
        self,
        image: np.ndarray,
        num_peaks: int = None,
        min_brightness: int = DEFAULT_MIN_BRIGHTNESS,
        circularity_threshold: float = DEFAULT_CIRCULARITY_THRESHOLD,
        color_threshold: float = DEFAULT_COLOR_THRESHOLD,
    ) -> List[Dict]:
        """
        Detect LEDs and measure their actual radius by analyzing where intensity
        drops to background levels.

        Args:
            image: Camera image
            num_peaks: Maximum number of LEDs to detect. Defaults to max_blobs.
            min_brightness: Minimum brightness for LED center
            circularity_threshold: Required circularity score (0-1)
            color_threshold: Minimum color score (0-100) to accept LED

        Returns:
            List of detected LEDs, each containing center, measured radius, circularity and color scores.
        """
        if num_peaks is None:
            num_peaks = self.max_blobs

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray_blur = cv2.GaussianBlur(gray, (5, 5), 0)

        candidate_points = self._find_local_maxima(gray_blur, num_peaks, min_brightness)

        if not candidate_points:
            return []

        valid_leds = []
        for point in candidate_points:
            # Skip if too close to already detected LEDs
            if self._is_too_close_to_existing_leds(
                point, valid_leds, use_measured_radius=True
            ):
                continue

            measured_radius = self._measure_led_radius(gray, point)
            circularity = self._calculate_circularity(gray, point, measured_radius)

            if circularity >= circularity_threshold and measured_radius > 5:
                color_score = self._calculate_color_score(image, point, measured_radius)
                if color_score >= color_threshold:
                    valid_leds.append(
                        {
                            "center": point,
                            "radius": measured_radius,
                            "circularity": circularity,
                            "brightness": gray[point[1], point[0]],
                            "color_score": color_score,
                        }
                    )

        # Sort by color score (best color match first) and limit to max_blobs
        sorted_leds = sorted(valid_leds, key=lambda x: x["color_score"], reverse=True)
        return sorted_leds[:num_peaks]

    def visualize_detections(
        self,
        image: np.ndarray,
        detections: List[Dict],
        window_name: str = "LED Detection",
        save_path: Path = None,
    ) -> None:
        """
        Display detected blobs

        Args:
            image: Original image
            detections: List of detected blobs
            window_name: Window title
            save_path: Optional path to save the annotated image
        """
        annotated_image = image.copy()

        for led in detections:
            center = led["center"]
            radius = led["radius"]

            # Draw circle
            cv2.circle(annotated_image, center, radius, (0, 0, 255), 2)

            # Mark center point
            cv2.circle(annotated_image, center, 3, (0, 255, 0), -1)

            # Add info text
            circularity_text = f"C:{led['circularity']:.2f}"
            cv2.putText(
                annotated_image,
                circularity_text,
                (center[0] + radius + 5, center[1]),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (255, 255, 0),
                1,
            )

        print(f"Detected {len(detections)} LEDs")

        # Save image if path provided
        if save_path:
            save_path.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(save_path), annotated_image)
            print(f"Saved to {save_path}")

        cv2.imshow(window_name, annotated_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def _is_too_close_to_existing_leds(
        self,
        point: Tuple[int, int],
        existing_leds: List[Dict],
        use_measured_radius: bool = False,  # TODO: check for removal
    ) -> bool:
        """
        Check if a point is too close to already detected bobs

        Args:
            point: (x, y) coordinates to check
            existing_leds: List of already detected leds
            use_measured_radius: If True, use actual measured radius from LED dict

        Returns:
            True if point overlaps with existing LEDs
        """
        for led in existing_leds:
            existing_center = led["center"]
            # Use measured radius if available, otherwise use expected radius
            radius = (
                led.get("radius", self.expected_radius)
                if use_measured_radius
                else self.expected_radius
            )
            min_distance = radius * 2.0

            distance = np.sqrt(
                (point[0] - existing_center[0]) ** 2
                + (point[1] - existing_center[1]) ** 2
            )
            if distance < min_distance:
                return True

        return False

    def _calculate_color_score(
        self, image: np.ndarray, center: Tuple[int, int], radius: int
    ) -> float:
        """
        Calculate how well the LED matches the target bright green color.

        Args:
            image: Original image
            center: (x, y) center coordinates
            radius: Radius to sample around center

        Returns:
            Color score (0-100), where higher means better color match
        """
        cx, cy = center
        height, width = image.shape[:2]

        # Create circular mask around led
        mask = np.zeros((height, width), dtype=np.uint8)
        cv2.circle(mask, center, radius, 255, -1)

        # Convert to hsv
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Check what percentage of the led's area is green
        color_mask = cv2.inRange(hsv, self.LOWER_GREEN, self.UPPER_GREEN)
        led_color_mask = cv2.bitwise_and(color_mask, mask)

        # Calculate percentage of green pixels in led area
        led_area = cv2.countNonZero(mask)
        green_pixels = cv2.countNonZero(led_color_mask)

        if led_area == 0:
            return 0.0

        color_score = (green_pixels / led_area) * 100
        return color_score

    def _calculate_circularity(
        self, gray: np.ndarray, center: Tuple[int, int], radius: int
    ) -> float:
        """
        Calculate circularity score based ight fall-off.

        Args:
            gray: Grayscale image
            center: Blob/circle center
            radius: Radius to sample around center

        Returns:
            Circularity score (0-1)
        """
        cx, cy = center
        height, width = gray.shape

        # Setup
        num_angles = 16  # Directions to sample
        angles = np.linspace(0, 2 * np.pi, num_angles, endpoint=False)
        sample_radii = [radius // 4, radius // 2, radius]

        # Collect radial intensity profiles
        radial_profiles = []
        for distance in sample_radii:
            intensities = []
            for angle in angles:
                x = int(cx + distance * np.cos(angle))
                y = int(cy + distance * np.sin(angle))

                if 0 <= x < width and 0 <= y < height:
                    intensities.append(gray[y, x])
                else:
                    intensities.append(0)

            if intensities:
                radial_profiles.append(intensities)

        # Calculate circularity from profile uniformity
        circularity_scores = []
        for profile in radial_profiles:
            if profile and np.mean(profile) > 0:
                # Coefficient of variation: lower values = more uniform
                cv = np.std(profile) / (np.mean(profile) + 1e-6)
                # Convert to 0-1 score (lower CV = higher circularity)
                score = np.exp(-cv)
                circularity_scores.append(score)

        return np.mean(circularity_scores) if circularity_scores else 0.0

    def _find_local_maxima(
        self, gray: np.ndarray, num_peaks: int, min_brightness: int
    ) -> List[Tuple[int, int]]:
        """
        Find the brightest local maxima above brightness threshold.

        Args:
            gray: Grayscale image
            num_peaks: Maximum number of peaks to find
            min_brightness: Minimum brightness threshold

        Returns:
            List of (x, y) coordinates of brightest points
        """
        kernel_size = 30  # Prevents multiple peaks per LED
        dilated = cv2.dilate(gray, np.ones((kernel_size, kernel_size), np.uint8))
        local_max_mask = (gray == dilated) & (gray >= min_brightness)

        y_coords, x_coords = np.where(local_max_mask)

        if len(y_coords) == 0:
            return []

        # Sort by brightness and return top candidates
        brightness_values = gray[y_coords, x_coords]
        sorted_indices = np.argsort(brightness_values)[::-1][: num_peaks * 2]

        return [(int(x_coords[i]), int(y_coords[i])) for i in sorted_indices]

    def _measure_led_radius(self, gray: np.ndarray, center: Tuple[int, int]) -> int:
        """
        Measure led radius by finding where intensity drops to background level.

        Args:
            gray: Grayscale image
            center: (x, y) center coordinates

        Returns:
            Measured radius in pixels
        """
        cx, cy = center
        center_brightness = gray[cy, cx]

        # Threshold: intensity drops to 30% of peak
        intensity_threshold = center_brightness * 0.3

        # Search outward from center
        max_search_radius = 50
        num_sample_points = 8

        for r in range(5, max_search_radius):
            angles = np.linspace(0, 2 * np.pi, num_sample_points, endpoint=False)
            intensities = []

            for angle in angles:
                x = int(cx + r * np.cos(angle))
                y = int(cy + r * np.sin(angle))

                if 0 <= x < gray.shape[1] and 0 <= y < gray.shape[0]:
                    intensities.append(gray[y, x])

            # Check if average intensity dropped below threshold
            if intensities and np.mean(intensities) < intensity_threshold:
                return r

        return self.expected_radius
