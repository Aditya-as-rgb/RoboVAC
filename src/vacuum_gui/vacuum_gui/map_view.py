# Copyright 2026 Aditya Choure
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""The map view: occupancy grid, LiDAR, plans, dirt and particles, drawn by us.

Everything is one QGraphicsScene in *map pixels* (1 scene unit = 1 occupancy cell), so
zoom and pan are native Qt behaviour and cost nothing per frame. Layers are separate
items that are only rebuilt when their message arrives (map 2 Hz, plans on change), which
is what keeps the CPU free for Gazebo.
"""

import math

from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import (QBrush, QColor, QFont, QImage, QPainter, QPainterPath, QPen,
                           QPixmap, QPolygonF)
from PySide6.QtWidgets import (QGraphicsPixmapItem, QGraphicsScene, QGraphicsTextItem,
                               QGraphicsView, QLabel, QVBoxLayout, QWidget)

DIRT_COLORS = {
    'crumbs': QColor(214, 158, 46),
    'dust': QColor(148, 216, 234),
    'spill': QColor(132, 94, 247),
    'hair': QColor(233, 120, 168),
}
DIRT_TRUTH = QColor(255, 146, 43)
MIN_SCALE = 0.05
MAX_SCALE = 40.0


class MapView(QGraphicsView):
    """Zoom/pan map with clicks that mean pose estimate, Nav2 goal or inspect."""

    pose_requested = Signal(float, float)
    goal_requested = Signal(float, float)
    inspect_requested = Signal(float, float)
    cursor_moved = Signal(float, float, bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setScene(QGraphicsScene(self))
        self.setRenderHints(QPainter.RenderHint.Antialiasing |
                            QPainter.RenderHint.SmoothPixmapTransform)
        self.setTransformationAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.ViewportAnchor.AnchorViewCenter)
        self.setDragMode(QGraphicsView.DragMode.NoDrag)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setBackgroundBrush(QBrush(QColor('#101317')))
        self.setFrameShape(QGraphicsView.Shape.NoFrame)
        self.setMouseTracking(True)

        self.grid = None
        self._map_item = None
        self._coverage_item = None
        self._cost_items = {}
        self._layers = {}
        self._layer_visible = {}
        self._dirty = set()
        self._detections = ()
        self._dirt_truth = ()
        self._robot_item = None
        self._trail_path = QPainterPath()
        self._trail_item = None
        self._click_mode = 'goal'
        self._press_pos = None
        self._panned = False
        self._fitted = False
        self._auto_fit = True
        self._pose = None

        self._hud = QLabel(self.viewport())
        self._hud.setObjectName('hint')
        self._hud.setStyleSheet('background: rgba(0,0,0,120); color: #dfe3e8; padding: 2px 6px;')
        self._hud.move(8, 8)
        self._hud.setText('no map yet')
        self._empty = QLabel('waiting for /map …\n\nStart the stack, or check the Health tab.',
                             self.viewport())
        self._empty.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._empty.setStyleSheet('color: #8b929b; font-size: 13px;')
        self._empty.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)

        for name in ('scan', 'plan_global', 'plan_local', 'coverage_plan', 'cleaned',
                     'dirt_detected', 'dirt_truth', 'particles', 'costmaps', 'trail'):
            self._layers[name] = []
            self._layer_visible[name] = name not in ('costmaps', 'plan_local')

    # ------------------------------------------------------------- geometry --
    def world_to_scene(self, x, y):
        if self.grid is None:
            return QPointF(x * 50.0, -y * 50.0)
        col, row = self.grid.world_to_pixel(x, y)
        return QPointF(col, row)

    def scene_to_world(self, point):
        if self.grid is None:
            return point.x() / 50.0, -point.y() / 50.0
        return self.grid.pixel_to_world(point.x(), point.y())

    def metres_to_px(self, metres):
        if self.grid is None:
            return metres * 50.0
        return metres / max(self.grid.resolution, 1e-6)

    def _item_visible(self, layer, item):
        item.setVisible(self._layer_visible.get(layer, True))
        item.setData(0, layer)
        self._layers.setdefault(layer, []).append(item)
        return item

    def set_layer_visible(self, layer, visible):
        self._layer_visible[layer] = bool(visible)
        for item in self._layers.get(layer, []):
            item.setVisible(bool(visible))

    def _clear_layer(self, layer):
        for item in self._layers.get(layer, []):
            if item.scene() is not None:
                self.scene().removeItem(item)
        self._layers[layer] = []
        self._dirty.discard(layer)

    # ------------------------------------------------------------------ map --
    def set_grid(self, grid):
        """New /map: rebuild the base image; keep zoom/pan after the first one."""
        first = self.grid is None
        rebox = (self.grid is None or grid.width != self.grid.width
                 or grid.height != self.grid.height or grid.resolution != self.grid.resolution)
        self.grid = grid
        from . import gridmap
        rgba = gridmap.rgba_from_grid(grid)
        image = QImage(rgba, grid.width, grid.height, grid.width * 4,
                       QImage.Format.Format_RGBA8888).copy()
        pixmap = QPixmap.fromImage(image)
        if self._map_item is None or rebox:
            if self._map_item is not None:
                self.scene().removeItem(self._map_item)
            self._map_item = QGraphicsPixmapItem(pixmap)
            self._map_item.setZValue(0)
            self.scene().addItem(self._map_item)
            self.scene().setSceneRect(QRectF(0, 0, grid.width, grid.height))
            self._coverage_item = None
        else:
            self._map_item.setPixmap(pixmap)
        self._empty.hide()
        self._update_hud()
        self._auto_fit = True
        if first or not self._fitted:
            self.fit()
            self._fitted = True

    def fit(self):
        if self.grid is None:
            return
        self.resetTransform()
        self.fitInView(QRectF(0, 0, self.grid.width, self.grid.height),
                       Qt.AspectRatioMode.KeepAspectRatio)

    def set_coverage(self, coverage_grid):
        if self.grid is None:
            return
        from . import gridmap
        if not self._layer_visible.get('cleaned', True):
            return
        rgba = gridmap.rgba_with_coverage(self.grid, coverage_grid)
        image = QImage(rgba, self.grid.width, self.grid.height, self.grid.width * 4,
                       QImage.Format.Format_RGBA8888).copy()
        pixmap = QPixmap.fromImage(image)
        if self._coverage_item is None:
            self._coverage_item = QGraphicsPixmapItem(pixmap)
            self._coverage_item.setZValue(1)
            self._coverage_item.setOpacity(0.85)
            self.scene().addItem(self._coverage_item)
            self._layers.setdefault('cleaned', []).append(self._coverage_item)
        else:
            self._coverage_item.setPixmap(pixmap)
        self._coverage_item.setVisible(self._layer_visible.get('cleaned', True))

    def set_costmap(self, kind, grid):
        if self.grid is None:
            return
        from . import gridmap
        arr = grid.data
        tinted = (arr > 0).astype('uint8') * 90
        rgba = gridmap.rgba_costmap(grid, tinted, color=(80, 130, 230, 110))
        image = QImage(rgba, grid.width, grid.height, grid.width * 4,
                       QImage.Format.Format_RGBA8888).copy()
        pixmap = QPixmap.fromImage(image)
        item = self._cost_items.get(kind)
        if item is None:
            item = QGraphicsPixmapItem(pixmap)
            item.setZValue(2)
            # a costmap has its own origin: place it in map pixels
            col, row = self.grid.world_to_pixel(grid.origin_x, grid.max_y)
            item.setPos(col, row)
            item.setScale(grid.resolution / self.grid.resolution)
            self.scene().addItem(item)
            self._cost_items[kind] = item
            self._layers.setdefault('costmaps', []).append(item)
        else:
            item.setPixmap(pixmap)
        item.setVisible(self._layer_visible.get('costmaps', False))

    # --------------------------------------------------------------- layers --
    def set_scan(self, scan):
        """Draw the scan as runs of nearby points.

        Dropped returns (glass, out of range) leave gaps in the angle sweep; connecting
        across a gap would draw straight rays across the room, so a jump longer than
        0.4 m starts a new polyline.
        """
        self._clear_layer('scan')
        if self.grid is None or len(scan.points) < 3:
            return
        gap = max(0.4, 10.0 * self.grid.resolution)
        path = QPainterPath()
        previous = None
        for x, y in scan.points:
            pt = self.world_to_scene(x, y)
            if previous is None or math.hypot(x - previous[0], y - previous[1]) > gap:
                path.moveTo(pt)
            else:
                path.lineTo(pt)
            previous = (x, y)
        item = self.scene().addPath(path, QPen(QColor(240, 82, 82, 190), 1.1))
        item.setZValue(6)
        self._item_visible('scan', item)

    def set_path(self, name, path_data):
        layer = 'plan_global' if name == 'global' else 'plan_local'
        self._clear_layer(layer)
        if self.grid is None:
            return
        pen = QPen(QColor(80, 170, 255, 230) if name == 'global'
                   else QColor(90, 230, 220, 200), 2.0)
        if name == 'local':
            pen.setStyle(Qt.PenStyle.DashLine)
        for polyline in path_data.polylines:
            if len(polyline) < 2:
                continue
            path = QPainterPath()
            first = True
            for x, y in polyline:
                pt = self.world_to_scene(x, y)
                if first:
                    path.moveTo(pt)
                    first = False
                else:
                    path.lineTo(pt)
            item = self.scene().addPath(path, pen)
            item.setZValue(7)
            self._item_visible(layer, item)

    def set_markers(self, kind, marker_set):
        layer = {'coverage': 'coverage_plan', 'dirt': 'dirt_truth',
                 'detected': 'dirt_detected'}.get(kind)
        if layer is None or self.grid is None:
            return
        self._clear_layer(layer)
        if kind == 'dirt':
            self._dirt_truth = ()
        truth = []
        font = QFont()
        font.setPointSizeF(max(5.0, min(9.0, 0.25 / max(self.grid.resolution, 1e-6))))
        for marker in marker_set.markers:
            color = QColor(*marker.color) if marker.color else QColor(200, 200, 200)
            if marker.kind == 'polyline' and len(marker.points) > 1:
                path = QPainterPath()
                first = True
                for x, y in marker.points:
                    pt = self.world_to_scene(x, y)
                    if first:
                        path.moveTo(pt)
                        first = False
                    else:
                        path.lineTo(pt)
                pen = QPen(color, max(1.2, self.metres_to_px(marker.size or 0.02)))
                item = self.scene().addPath(path, pen)
            elif marker.kind == 'text':
                item = QGraphicsTextItem(marker.label)
                item.setDefaultTextColor(color)
                item.setFont(font)
                item.setPos(self.world_to_scene(marker.x, marker.y))
                self.scene().addItem(item)
            else:
                radius = max(2.0, self.metres_to_px((marker.size or 0.2) / 2.0))
                centre = self.world_to_scene(marker.x, marker.y)
                item = self.scene().addEllipse(
                    QRectF(centre.x() - radius, centre.y() - radius, radius * 2, radius * 2),
                    QPen(color.lighter(130), 1.0), QBrush(QColor(color.red(), color.green(),
                                                                 color.blue(), 110)))
                if kind == 'dirt':
                    truth.append((marker.x, marker.y, marker.label))
            item.setZValue(8 if kind != 'coverage' else 7)
            self._item_visible(layer, item)
        if kind == 'dirt':
            self._dirt_truth = tuple(truth)

    def set_particles(self, cloud):
        self._clear_layer('particles')
        if self.grid is None or not cloud.poses:
            return
        path = QPainterPath()
        radius = max(0.8, self.metres_to_px(0.015))
        for x, y, _yaw in cloud.poses:
            pt = self.world_to_scene(x, y)
            path.addEllipse(QRectF(pt.x() - radius, pt.y() - radius, radius * 2, radius * 2))
        item = self.scene().addPath(path, QPen(Qt.PenStyle.NoPen),
                                    QBrush(QColor(120, 200, 255, 150)))
        item.setZValue(6)
        self._item_visible('particles', item)

    def set_detections(self, detections):
        self._detections = detections.spots
        self._clear_layer('dirt_detected')
        if self.grid is None:
            return
        font = QFont()
        font.setPointSizeF(max(5.0, min(9.0, 0.22 / max(self.grid.resolution, 1e-6))))
        for spot in detections.spots:
            color = DIRT_COLORS.get(spot.kind, QColor(230, 230, 230))
            radius = max(2.5, self.metres_to_px(0.14))
            centre = self.world_to_scene(spot.x, spot.y)
            item = self.scene().addEllipse(
                QRectF(centre.x() - radius, centre.y() - radius, radius * 2, radius * 2),
                QPen(color.lighter(120), 1.4),
                QBrush(QColor(color.red(), color.green(), color.blue(), 90)))
            item.setZValue(9)
            self._item_visible('dirt_detected', item)
            label = QGraphicsTextItem('%s %.2f' % (spot.kind, spot.conf))
            label.setDefaultTextColor(color.lighter(130))
            label.setFont(font)
            label.setPos(centre.x() + radius, centre.y() - radius)
            label.setZValue(10)
            self.scene().addItem(label)
            self._item_visible('dirt_detected', label)
            item.setToolTip('%s  conf %.2f  hits %d' % (spot.kind, spot.conf, spot.hits))

    def set_pose(self, pose):
        self._pose = pose
        if self.grid is None:
            return
        if self._robot_item is None:
            self._robot_item = self.scene().addPolygon(
                QPolygonF(), QPen(QColor('#7bd88f'), 2.0), QBrush(QColor('#2f9e44')))
            self._robot_item.setZValue(11)
        size = max(5.0, self.metres_to_px(0.5))
        half = size / 2.0
        triangle = QPolygonF([
            QPointF(half, 0.0),
            QPointF(-half * 0.7, half * 0.65),
            QPointF(-half * 0.35, 0.0),
            QPointF(-half * 0.7, -half * 0.65),
        ])
        self._robot_item.setPolygon(triangle)
        centre = self.world_to_scene(pose.x, pose.y)
        self._robot_item.setPos(centre)
        # scene y grows downwards, world yaw grows counter-clockwise: flip the angle
        self._robot_item.setRotation(-math.degrees(pose.yaw))

        if self._trail_item is None:
            self._trail_item = self.scene().addPath(QPainterPath(),
                                                    QPen(QColor(123, 216, 143, 120), 1.0))
            self._trail_item.setZValue(5)
            self._layers.setdefault('trail', []).append(self._trail_item)
            self._trail_item.setVisible(self._layer_visible.get('trail', True))
        if self._trail_path.elementCount() == 0:
            self._trail_path.moveTo(centre)
        else:
            last = self._trail_path.currentPosition()
            if (centre - last).manhattanLength() > max(1.0, self.metres_to_px(0.05)):
                self._trail_path.lineTo(centre)
                self._trail_item.setPath(self._trail_path)

    def clear_trail(self):
        self._trail_path = QPainterPath()
        if self._trail_item is not None:
            self._trail_item.setPath(self._trail_path)

    # ----------------------------------------------------------- interaction --
    def set_click_mode(self, mode):
        self._click_mode = mode
        cursors = {'goal': Qt.CursorShape.CrossCursor, 'pose': Qt.CursorShape.CrossCursor,
                   'inspect': Qt.CursorShape.PointingHandCursor, 'pan': Qt.CursorShape.OpenHandCursor}
        self.viewport().setCursor(cursors.get(mode, Qt.CursorShape.CrossCursor))
        self._update_hud()

    def _update_hud(self, world=None):
        mode = {'goal': 'click: Nav2 goal', 'pose': 'click: 2D pose estimate',
                'inspect': 'click: inspect dirt', 'pan': 'drag to pan'}.get(self._click_mode, '')
        text = mode
        if world is not None:
            text += '   x %.2f  y %.2f m' % world
        self._hud.setText(text)
        self._hud.adjustSize()

    def mousePressEvent(self, event):      # noqa: N802
        if event.button() == Qt.MouseButton.LeftButton:
            self._press_pos = event.position()
            self._panned = False
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):      # noqa: N802
        world = self.scene_to_world(self.mapToScene(event.position().toPoint()))
        self.cursor_moved.emit(world[0], world[1], self.grid is not None)
        self._update_hud(world)
        if self._press_pos is not None and event.buttons() & Qt.MouseButton.LeftButton:
            delta = event.position() - self._press_pos
            if abs(delta.x()) > 4 or abs(delta.y()) > 4:
                self._panned = True
                self._auto_fit = False
                self.setCursor(Qt.CursorShape.ClosedHandCursor)
                self.horizontalScrollBar().setValue(
                    self.horizontalScrollBar().value() - int(delta.x()))
                self.verticalScrollBar().setValue(
                    self.verticalScrollBar().value() - int(delta.y()))
                self._press_pos = event.position()
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):      # noqa: N802
        if event.button() == Qt.MouseButton.LeftButton:
            panned = self._panned
            self._press_pos = None
            self._panned = False
            self.set_click_mode(self._click_mode)
            if not panned and self.grid is not None:
                scene_pos = self.mapToScene(event.position().toPoint())
                x, y = self.scene_to_world(scene_pos)
                if self._click_mode == 'pose':
                    self.pose_requested.emit(x, y)
                elif self._click_mode == 'inspect':
                    self.inspect_requested.emit(x, y)
                else:
                    self.goal_requested.emit(x, y)
        super().mouseReleaseEvent(event)

    def wheelEvent(self, event):      # noqa: N802
        factor = 1.18 if event.angleDelta().y() > 0 else 1 / 1.18
        self._auto_fit = False
        current = self.transform().m11()
        target = current * factor
        if MIN_SCALE <= target <= MAX_SCALE:
            self.scale(factor, factor)
        event.accept()

    def resizeEvent(self, event):      # noqa: N802
        super().resizeEvent(event)
        self._empty.setGeometry(0, 0, self.viewport().width(), self.viewport().height())
        if self.grid is not None and self._auto_fit:
            self.fit()

    # ---------------------------------------------------------------- extras --
    def nearest_detection(self, x, y, max_distance=0.4):
        best, found = max_distance, None
        for spot in self._detections:
            d = math.hypot(spot.x - x, spot.y - y)
            if d <= best:
                best, found = d, spot
        return found

    def nearest_truth(self, x, y):
        best, found = float('inf'), None
        for tx, ty, label in self._dirt_truth:
            d = math.hypot(tx - x, ty - y)
            if d < best:
                best, found = d, (tx, ty, label)
        return found, best

    def grab_png(self, path):
        """Screenshot of what is on screen (for slides)."""
        pixmap = self.viewport().grab()
        return pixmap.save(path, 'PNG')


class MapPanel(QWidget):
    """MapView plus a small header line (map size / resolution)."""

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        self.view = MapView(self)
        self.info = QLabel('no map')
        self.info.setObjectName('hint')
        self.info.setContentsMargins(8, 2, 8, 2)
        layout.addWidget(self.view, 1)
        layout.addWidget(self.info)

    def set_grid(self, grid):
        self.view.set_grid(grid)
        self.info.setText('%d x %d cells  %.3f m/cell  origin (%.2f, %.2f)  frame %s'
                          % (grid.width, grid.height, grid.resolution,
                             grid.origin_x, grid.origin_y, grid.frame))
