"""Unit tests for the review overlay and HTML index."""

from hexapod_perception.review_overlay import draw_overlay, PANEL_H
from hexapod_perception.review_recorder import write_index_html
import numpy as np

H, W = 480, 640


def blank():
    return np.zeros((H, W, 3), dtype=np.uint8)


def gauge(img):
    """Crop the velocity gauge region (bottom-right of the camera image)."""
    return img[H - 80:H, W - 80:W]


def test_overlay_adds_panel_and_keeps_input():
    img = blank()
    out = draw_overlay(img, 0.1, 0.0, 0.05, (2.0, 0.5), 3.0)
    assert out.shape == (H + PANEL_H, W, 3)
    assert not img.any()            # input untouched
    assert out[H:].any()            # panel has text


def test_forward_arrow_points_up_reverse_points_down():
    fwd = gauge(draw_overlay(blank(), 0.3, 0.0, 0.0, None, 0.0))
    rev = gauge(draw_overlay(blank(), -0.3, 0.0, 0.0, None, 0.0))
    # Gauge centre is 36 px above the image bottom -> row 44 of the 80 px crop.
    green = (fwd[:, :, 1] > 150) & (fwd[:, :, 2] < 120)
    red = (rev[:, :, 2] > 150) & (rev[:, :, 1] < 120)
    assert green[:40].sum() > green[48:].sum()
    assert red[48:].sum() > red[:40].sum()


def test_left_turn_arrow_tilts_left():
    out = gauge(draw_overlay(blank(), 0.3, 0.6, 0.0, None, 0.0))
    green = (out[:, :, 1] > 150) & (out[:, :, 2] < 120)
    cols = np.nonzero(green[:40])[1]
    assert cols.mean() < 44  # gauge centre column is 44 in the crop


def test_turn_in_place_draws_arc_and_no_cmd_is_handled():
    spin = gauge(draw_overlay(blank(), 0.0, 0.4, 0.0, None, 0.0))
    assert spin.any()
    none = draw_overlay(blank(), None, None, None, None, 0.0)
    assert none.shape == (H + PANEL_H, W, 3)


def test_index_html_lists_rows(tmp_path):
    rows = [{'frame': 0, 'time_s': '0.000', 'lin_x': '0.000', 'ang_z': '0.000',
             'cmd_age_s': '0.05', 'target_forward': '', 'target_left': '',
             'image': 'frames/frame_000000.jpg'},
            {'frame': 1, 'time_s': '0.200', 'lin_x': '0.250', 'ang_z': '-0.100',
             'cmd_age_s': '0.02', 'target_forward': '2.10', 'target_left': '0.30',
             'image': 'frames/frame_000001.jpg'}]
    path = tmp_path / 'index.html'
    write_index_html(str(path), rows)
    page = path.read_text()
    assert page.count('<tr class=') == 2
    assert 'class="mv"' in page and '2.10 m fwd' in page
