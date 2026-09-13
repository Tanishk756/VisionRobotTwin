"""ArUco Marker Generator Utility.

Generates printable ArUco fiducial markers for the VisionRobotTwin system:
- Marker ID 0: Manual Robot Control Target
- Marker ID 1: Autonomous Pick Waypoint
- Marker ID 2: Autonomous Place Waypoint

Also produces a combined printable multi-marker sheet with human-readable labels.
"""

import sys
from pathlib import Path
import cv2
import numpy as np

# Ensure project root is in sys.path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from config.settings import ArUcoConfig
from utils.logger import setup_logger, get_logger

setup_logger()
logger = get_logger("Tools.GenerateMarkers")


def generate_single_marker(
    dictionary: cv2.aruco.Dictionary,
    marker_id: int,
    pixel_size: int = 500,
    border_bits: int = 1,
) -> np.ndarray:
    """Generates a high-resolution single ArUco marker image."""
    img = np.zeros((pixel_size, pixel_size), dtype=np.uint8)
    cv2.aruco.generateImageMarker(dictionary, marker_id, pixel_size, img, border_bits)
    return img


def create_marker_card(
    marker_img: np.ndarray,
    marker_id: int,
    semantic_label: str,
    target_size_cm: float = 5.0,
) -> np.ndarray:
    """Wraps marker in a bordered printable card with title, ID, and physical dimensions."""
    m_h, m_w = marker_img.shape
    pad_top = 80
    pad_bottom = 90
    pad_sides = 60

    card_w = m_w + 2 * pad_sides
    card_h = m_h + pad_top + pad_bottom

    card = np.full((card_h, card_w, 3), 255, dtype=np.uint8)

    # Insert marker
    card[pad_top : pad_top + m_h, pad_sides : pad_sides + m_w] = cv2.cvtColor(
        marker_img, cv2.COLOR_GRAY2BGR
    )

    # Outer cut border
    cv2.rectangle(card, (5, 5), (card_w - 5, card_h - 5), (180, 180, 180), 1, cv2.LINE_AA)

    # Header title
    cv2.putText(
        card,
        f"VisionRobotTwin - Marker ID {marker_id}",
        (pad_sides, 45),
        cv2.FONT_HERSHEY_DUPLEX,
        0.85,
        (20, 20, 20),
        2,
        cv2.LINE_AA,
    )

    # Footer semantic label and size instructions
    cv2.putText(
        card,
        f"ROLE: {semantic_label.upper()}",
        (pad_sides, card_h - 50),
        cv2.FONT_HERSHEY_DUPLEX,
        0.75,
        (0, 100, 200),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        card,
        f"Print size: {target_size_cm:.1f} cm x {target_size_cm:.1f} cm (DICT_4X4_50)",
        (pad_sides, card_h - 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (100, 100, 100),
        1,
        cv2.LINE_AA,
    )

    return card


def generate_all_markers(output_dir: Path = Path("assets/markers")) -> None:
    """Generates and saves all project markers and combined printable sheet."""
    output_dir.mkdir(parents=True, exist_ok=True)
    config = ArUcoConfig()

    dict_attr = getattr(cv2.aruco, config.dictionary_name, cv2.aruco.DICT_4X4_50)
    dictionary = cv2.aruco.getPredefinedDictionary(dict_attr)

    marker_defs = [
        (config.target_marker_id, "Manual Robot Target"),
        (config.pick_marker_id, "Autonomous Pick Location"),
        (config.place_marker_id, "Autonomous Place Location"),
    ]

    cards = []
    for mid, label in marker_defs:
        raw_marker = generate_single_marker(dictionary, mid, pixel_size=500)
        
        # Save raw marker
        raw_path = output_dir / f"marker_{mid}_raw.png"
        cv2.imwrite(str(raw_path), raw_marker)

        # Save printable card
        card = create_marker_card(raw_marker, mid, label, target_size_cm=config.marker_size_m * 100)
        card_path = output_dir / f"marker_{mid}_{label.lower().replace(' ', '_')}.png"
        cv2.imwrite(str(card_path), card)
        cards.append(card)

        logger.info(f"Generated Marker ID {mid} ('{label}') -> {card_path}")

    # Create combined 3-marker printable overview sheet
    max_w = max(c.shape[1] for c in cards)
    gap = 20
    total_h = sum(c.shape[0] for c in cards) + gap * (len(cards) + 1)
    combined = np.full((total_h, max_w + 40, 3), 255, dtype=np.uint8)

    y_offset = gap
    for card in cards:
        ch, cw = card.shape[:2]
        x_offset = (combined.shape[1] - cw) // 2
        combined[y_offset : y_offset + ch, x_offset : x_offset + cw] = card
        y_offset += ch + gap

    sheet_path = output_dir / "all_markers_sheet.png"
    cv2.imwrite(str(sheet_path), combined)
    logger.info(f"Combined marker sheet saved -> {sheet_path}")


if __name__ == "__main__":
    generate_all_markers()
