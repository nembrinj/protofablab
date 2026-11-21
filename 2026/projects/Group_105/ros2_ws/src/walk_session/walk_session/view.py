"""
The session panel shown in RViz: the walk in progress.

The panel is fixed in the top left corner of the RViz view
(rviz_2d_overlay_plugins), one item per line: RViz's own text markers draw
spaces far too wide.

This module has no ROS imports, so it can be tested without ROS.
"""

import html

# The panel, in pixels of the RViz view.
PANEL_WIDTH_PX = 200
PANEL_MARGIN_PX = 10  # from the edges of the view
PANEL_LINE_PX = 20
PANEL_TEXT_PT = 11


def panel_lines(progress, walks):
    """Return the panel's lines: the walk in progress, or how many walks are done."""
    if progress is None:
        return ['No walk yet'] if walks == 0 else [f'{count(walks, "walk")} done']
    return [
        f'Walk {progress.number}: {"stopped" if progress.stopped else "walking"}',
        f'{count(progress.steps, "step")}, {count(progress.stops, "stop")}',
    ]


def panel_text(lines):
    """Join the lines for the panel, which reads its text as HTML."""
    return '\n'.join(html.escape(line, quote=False) for line in lines)


def panel_height_px(lines):
    return len(lines) * PANEL_LINE_PX + PANEL_LINE_PX // 2


def count(number, thing):
    return f'{number} {thing}' + ('' if number == 1 else 's')
