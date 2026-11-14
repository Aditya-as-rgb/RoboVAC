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

"""Colour themes for the panel, switchable at runtime.

A theme is one small dict of base colours. Everything else (hover and pressed shades,
the accent variants) is derived from it, so adding a theme means adding ~12 hex values.
``system`` is not a theme of its own: it follows the desktop's light/dark setting and
picks ``graphite`` or ``daylight``.

The stylesheet gives every button a hover, pressed, focus, busy and result state; the
busy / flash / result states are set by ``feedback.py`` through dynamic properties.
"""

from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QPalette
from PySide6.QtWidgets import QApplication

SYSTEM = 'system'

# key -> (label, is_dark, colours). ``accent_text`` is the text colour on an accent fill.
THEMES = {
    'graphite': ('Graphite', True, {
        'window': '#1c1f24', 'base': '#14171b', 'panel': '#23272e', 'border': '#333941',
        'text': '#e6e6e6', 'subtext': '#a0a4a8', 'caption': '#8b929b',
        'accent': '#4d9de0', 'accent_text': '#0b1620',
        'ok': '#2f9e44', 'warn': '#e8a33d', 'fail': '#e03131'}),
    'midnight': ('Midnight', True, {
        'window': '#0b1220', 'base': '#070d18', 'panel': '#121c2f', 'border': '#22314d',
        'text': '#dbe7ff', 'subtext': '#9db0d1', 'caption': '#7a8db0',
        'accent': '#22d3ee', 'accent_text': '#04222a',
        'ok': '#34d399', 'warn': '#fbbf24', 'fail': '#f87171'}),
    'nord': ('Nord', True, {
        'window': '#2e3440', 'base': '#272c36', 'panel': '#3b4252', 'border': '#4c566a',
        'text': '#eceff4', 'subtext': '#d8dee9', 'caption': '#9aa5b8',
        'accent': '#88c0d0', 'accent_text': '#1f2430',
        'ok': '#a3be8c', 'warn': '#ebcb8b', 'fail': '#bf616a'}),
    'dracula': ('Dracula', True, {
        'window': '#282a36', 'base': '#1e1f29', 'panel': '#343746', 'border': '#44475a',
        'text': '#f8f8f2', 'subtext': '#c5c8d8', 'caption': '#8a8fb0',
        'accent': '#bd93f9', 'accent_text': '#1b1530',
        'ok': '#50fa7b', 'warn': '#f1fa8c', 'fail': '#ff5555'}),
    'mocha': ('Mocha', True, {
        'window': '#1e1e2e', 'base': '#181825', 'panel': '#313244', 'border': '#45475a',
        'text': '#cdd6f4', 'subtext': '#a6adc8', 'caption': '#7f849c',
        'accent': '#89b4fa', 'accent_text': '#11111b',
        'ok': '#a6e3a1', 'warn': '#f9e2af', 'fail': '#f38ba8'}),
    'high-contrast': ('High contrast', True, {
        'window': '#000000', 'base': '#000000', 'panel': '#0d0d0d', 'border': '#8a8a8a',
        'text': '#ffffff', 'subtext': '#e6e6e6', 'caption': '#c4c4c4',
        'accent': '#ffd60a', 'accent_text': '#000000',
        'ok': '#3ddc84', 'warn': '#ffb000', 'fail': '#ff453a'}),
    'daylight': ('Daylight', False, {
        'window': '#f2f4f7', 'base': '#ffffff', 'panel': '#ffffff', 'border': '#d0d7de',
        'text': '#1b1f24', 'subtext': '#4b535c', 'caption': '#5c6670',
        'accent': '#1971c2', 'accent_text': '#ffffff',
        'ok': '#2b8a3e', 'warn': '#b7791f', 'fail': '#c92a2a'}),
    'solarized-light': ('Solarized light', False, {
        'window': '#f5eedb', 'base': '#fdf6e3', 'panel': '#fdf6e3', 'border': '#d8d0b8',
        'text': '#4a5f66', 'subtext': '#586e75', 'caption': '#6f8187',
        'accent': '#268bd2', 'accent_text': '#fdf6e3',
        'ok': '#738a05', 'warn': '#a67c00', 'fail': '#dc322f'}),
}

DEFAULT_DARK = 'graphite'
DEFAULT_LIGHT = 'daylight'

# the two names settings.json used before themes existed
LEGACY = {'dark': DEFAULT_DARK, 'light': DEFAULT_LIGHT}


def theme_choices():
    """[(key, label)] for the drop-down, 'follow the desktop' first."""
    return [(SYSTEM, 'System')] + [(key, spec[0]) for key, spec in THEMES.items()]


def normalize(mode):
    """Any stored value -> a key of THEMES or 'system'."""
    mode = str(mode or SYSTEM).strip().lower()
    mode = LEGACY.get(mode, mode)
    return mode if mode in THEMES or mode == SYSTEM else SYSTEM


def label(mode):
    mode = normalize(mode)
    return 'System' if mode == SYSTEM else THEMES[mode][0]


def _shade(color, dark, amount):
    """Lighten a dark theme / darken a light one by ``amount`` percent (hover feel)."""
    c = QColor(color)
    return (c.lighter(100 + amount) if dark else c.darker(100 + amount)).name()


def colors(key):
    """The base colours of theme ``key`` plus every derived shade the stylesheet uses."""
    _label, dark, base = THEMES[key]
    out = dict(base)
    out['dark'] = dark
    out['hover'] = _shade(base['panel'], dark, 14 if dark else 4)
    out['pressed'] = QColor(base['panel']).darker(118 if dark else 112).name()
    out['accent_hover'] = QColor(base['accent']).lighter(112).name()
    out['accent_pressed'] = QColor(base['accent']).darker(125).name()
    out['fail_hover'] = QColor(base['fail']).lighter(112).name()
    out['fail_pressed'] = QColor(base['fail']).darker(125).name()
    accent = QColor(base['accent'])
    out['busy_bg'] = (accent.darker(300) if dark else accent.lighter(185)).name()
    out['window_dim'] = QColor(base['window']).darker(106).name()
    out['on_fail'] = '#ffffff'
    return out


STYLESHEET = """
QMainWindow, QDialog {{ background: {window}; }}
QWidget {{ color: {text}; }}
QToolTip {{ background: {panel}; color: {text}; border: 1px solid {border}; padding: 5px 7px;
           border-radius: 4px; }}

QToolBar {{ background: {panel}; border: none; border-bottom: 1px solid {border};
           spacing: 6px; padding: 6px 10px; }}
QToolBar::separator {{ background: {border}; width: 1px; margin: 5px 8px; }}
QToolButton {{ background: transparent; color: {text}; border: 1px solid transparent;
              border-radius: 6px; padding: 6px 11px; }}
QToolButton:hover {{ background: {hover}; border-color: {border}; }}
QToolButton:pressed {{ background: {pressed}; border-color: {accent};
                      padding-top: 7px; padding-bottom: 5px; }}
QToolButton:checked {{ background: {pressed}; border-color: {accent}; }}
QToolButton:disabled {{ color: {caption}; }}
QToolButton[flash="true"] {{ background: {accent}; color: {accent_text}; border-color: {accent}; }}

QStatusBar {{ background: {panel}; border-top: 1px solid {border}; color: {subtext}; }}
QStatusBar::item {{ border: none; }}
QMenu {{ background: {panel}; border: 1px solid {border}; padding: 4px; }}
QMenu::item {{ padding: 6px 22px; border-radius: 4px; }}
QMenu::item:selected {{ background: {accent}; color: {accent_text}; }}

QGroupBox {{ border: 1px solid {border}; border-radius: 8px; margin-top: 16px;
            padding: 10px 8px 8px 8px; font-weight: 600; background: transparent; }}
QGroupBox::title {{ subcontrol-origin: margin; left: 12px; padding: 0 5px; color: {subtext}; }}
QDockWidget {{ color: {subtext}; titlebar-close-icon: none; titlebar-normal-icon: none; }}
QDockWidget::title {{ background: {panel}; padding: 6px 10px; border: 1px solid {border};
                     border-bottom: none; font-weight: 600; }}
QTabWidget::pane {{ border: 1px solid {border}; border-radius: 6px; top: -1px; }}
QTabBar::tab {{ background: transparent; padding: 6px 12px; color: {subtext};
               border-bottom: 2px solid transparent; }}
QTabBar::tab:hover {{ color: {text}; background: {hover}; }}
QTabBar::tab:selected {{ color: {text}; border-bottom: 2px solid {accent}; }}

QPushButton {{ background: {panel}; color: {text}; border: 1px solid {border};
              border-radius: 6px; padding: 6px 13px; min-height: 18px; }}
QPushButton:hover {{ background: {hover}; border-color: {accent}; }}
QPushButton:pressed {{ background: {pressed}; border-color: {accent};
                      padding-top: 7px; padding-bottom: 5px; }}
QPushButton:focus {{ border-color: {accent}; }}
QPushButton:checked {{ background: {pressed}; border-color: {accent}; }}
QPushButton:disabled {{ color: {caption}; background: {window}; border-color: {border}; }}
QPushButton:flat {{ background: transparent; border-color: transparent; }}
QPushButton:flat:hover {{ background: {hover}; border-color: {border}; }}

QPushButton#primary {{ background: {accent}; color: {accent_text}; border: 1px solid {accent};
                      font-weight: 600; }}
QPushButton#primary:hover {{ background: {accent_hover}; border-color: {accent_hover}; }}
QPushButton#primary:pressed {{ background: {accent_pressed}; border-color: {accent_pressed};
                              padding-top: 7px; padding-bottom: 5px; }}
QPushButton#primary:disabled {{ background: {window}; color: {caption}; border-color: {border}; }}

QPushButton#danger {{ background: {fail}; color: {on_fail}; border: 1px solid {fail};
                     font-weight: 700; }}
QPushButton#danger:hover {{ background: {fail_hover}; border-color: {fail_hover}; }}
QPushButton#danger:pressed {{ background: {fail_pressed}; border-color: {fail_pressed};
                             padding-top: 7px; padding-bottom: 5px; }}
QPushButton#danger:checked {{ background: #ff2020; border: 2px solid #ffd43b; }}
QPushButton#danger:disabled {{ background: {window}; color: {caption}; border-color: {border}; }}

QPushButton[flash="true"], QPushButton#primary[flash="true"], QPushButton#danger[flash="true"] {{
    border: 2px solid {accent}; background: {pressed}; color: {text}; }}
QPushButton[busy="true"], QPushButton[busy="true"]:disabled,
QPushButton#primary[busy="true"], QPushButton#primary[busy="true"]:disabled,
QPushButton#danger[busy="true"], QPushButton#danger[busy="true"]:disabled {{
    background: {busy_bg}; color: {text}; border: 1px solid {accent}; font-weight: 600; }}
QPushButton[result="ok"], QPushButton#primary[result="ok"], QPushButton#danger[result="ok"] {{
    border: 2px solid {ok}; }}
QPushButton[result="fail"], QPushButton#primary[result="fail"],
QPushButton#danger[result="fail"] {{ border: 2px solid {fail}; }}

QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox, QPlainTextEdit, QTextEdit {{
    background: {base}; border: 1px solid {border}; border-radius: 5px; padding: 4px 7px;
    selection-background-color: {accent}; selection-color: {accent_text}; }}
QLineEdit:hover, QSpinBox:hover, QDoubleSpinBox:hover, QComboBox:hover {{ border-color: {caption}; }}
QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus,
QPlainTextEdit:focus, QTextEdit:focus {{ border-color: {accent}; }}
QLineEdit:disabled, QSpinBox:disabled, QDoubleSpinBox:disabled, QComboBox:disabled {{
    color: {caption}; background: {window}; }}
QComboBox QAbstractItemView {{ background: {panel}; border: 1px solid {border}; outline: 0;
                              selection-background-color: {accent};
                              selection-color: {accent_text}; padding: 3px; }}
QCheckBox, QRadioButton {{ spacing: 7px; }}
QCheckBox::indicator, QRadioButton::indicator {{ width: 15px; height: 15px; }}

QHeaderView::section {{ background: {panel}; border: none; border-bottom: 1px solid {border};
                       padding: 5px; color: {subtext}; font-weight: 600; }}
QTableWidget, QTreeWidget, QListWidget {{ background: {base}; border: 1px solid {border};
                                         border-radius: 6px; gridline-color: {border}; }}
QListWidget::item, QTableWidget::item {{ padding: 3px; }}
QListWidget::item:hover {{ background: {hover}; }}
QListWidget::item:selected, QTableWidget::item:selected {{ background: {accent};
                                                           color: {accent_text}; }}

QProgressBar {{ background: {base}; border: 1px solid {border}; border-radius: 7px;
               text-align: center; min-height: 18px; color: {text}; }}
QProgressBar::chunk {{ background: {accent}; border-radius: 6px; }}

QLabel#caption {{ color: {caption}; font-size: 9px; letter-spacing: 1px; }}
QLabel#value {{ font-size: 15px; font-weight: 600; }}
QLabel#section {{ color: {subtext}; font-size: 10px; font-weight: 700; letter-spacing: 1px;
                 margin-top: 6px; }}
QLabel#hint {{ color: {caption}; font-size: 11px; }}
QLabel#badge {{ background: {warn}; color: #1b1b1b; border-radius: 8px; padding: 2px 9px;
               font-size: 10px; font-weight: 700; }}
QLabel#badgeOk {{ background: transparent; color: {caption}; padding: 2px 9px; font-size: 10px; }}
QFrame#hline {{ color: {border}; max-height: 1px; }}
QSplitter::handle {{ background: {border}; }}

QScrollBar:vertical {{ background: transparent; width: 11px; margin: 2px; }}
QScrollBar::handle:vertical {{ background: {border}; border-radius: 4px; min-height: 26px; }}
QScrollBar::handle:vertical:hover {{ background: {caption}; }}
QScrollBar:horizontal {{ background: transparent; height: 11px; margin: 2px; }}
QScrollBar::handle:horizontal {{ background: {border}; border-radius: 4px; min-width: 26px; }}
QScrollBar::handle:horizontal:hover {{ background: {caption}; }}
QScrollBar::add-line, QScrollBar::sub-line {{ width: 0; height: 0; }}
QScrollBar::add-page, QScrollBar::sub-page {{ background: transparent; }}

QFrame#toast {{ background: {panel}; border: 1px solid {border}; border-left: 4px solid {accent};
               border-radius: 8px; }}
QFrame#toast[level="busy"] {{ border-left-color: {accent}; }}
QFrame#toast[level="ok"] {{ border-left-color: {ok}; }}
QFrame#toast[level="warn"] {{ border-left-color: {warn}; }}
QFrame#toast[level="error"] {{ border-left-color: {fail}; }}
QLabel#toastText {{ color: {text}; background: transparent; }}
QLabel#toastIcon {{ color: {accent}; font-size: 15px; font-weight: 700; background: transparent; }}
QFrame#toast[level="ok"] QLabel#toastIcon {{ color: {ok}; }}
QFrame#toast[level="warn"] QLabel#toastIcon {{ color: {warn}; }}
QFrame#toast[level="error"] QLabel#toastIcon {{ color: {fail}; }}
"""


def _palette(c):
    pal = QPalette()
    pal.setColor(QPalette.ColorRole.Window, QColor(c['window']))
    pal.setColor(QPalette.ColorRole.WindowText, QColor(c['text']))
    pal.setColor(QPalette.ColorRole.Base, QColor(c['base']))
    pal.setColor(QPalette.ColorRole.AlternateBase, QColor(c['panel']))
    pal.setColor(QPalette.ColorRole.Text, QColor(c['text']))
    pal.setColor(QPalette.ColorRole.Button, QColor(c['panel']))
    pal.setColor(QPalette.ColorRole.ButtonText, QColor(c['text']))
    pal.setColor(QPalette.ColorRole.Highlight, QColor(c['accent']))
    pal.setColor(QPalette.ColorRole.HighlightedText, QColor(c['accent_text']))
    pal.setColor(QPalette.ColorRole.ToolTipBase, QColor(c['panel']))
    pal.setColor(QPalette.ColorRole.ToolTipText, QColor(c['text']))
    pal.setColor(QPalette.ColorRole.PlaceholderText, QColor(c['caption']))
    pal.setColor(QPalette.ColorRole.Link, QColor(c['accent']))
    pal.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Text, QColor(c['caption']))
    pal.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ButtonText,
                 QColor(c['caption']))
    return pal


def system_scheme(app):
    """'dark' or 'light' as the desktop reports it (defaults to dark)."""
    try:
        scheme = app.styleHints().colorScheme()
        if scheme == Qt.ColorScheme.Light:
            return 'light'
        if scheme == Qt.ColorScheme.Dark:
            return 'dark'
    except AttributeError:
        pass
    return 'dark'


def resolved(app, mode):
    """The concrete theme key for a stored mode ('system' follows the desktop)."""
    mode = normalize(mode)
    if mode == SYSTEM:
        return DEFAULT_LIGHT if system_scheme(app) == 'light' else DEFAULT_DARK
    return mode


def apply(app, mode=SYSTEM):
    """Apply a theme; returns the concrete theme key that was used."""
    key = resolved(app, mode)
    c = colors(key)
    try:
        if app.style().objectName().lower() != 'fusion':
            app.setStyle('Fusion')      # the stylesheet looks the same on every desktop
    except AttributeError:
        pass
    app.setPalette(_palette(c))
    app.setStyleSheet(STYLESHEET.format(**c))
    return key


def connect_system_theme(app, callback):
    """Call ``callback()`` when the desktop switches between light and dark."""
    try:
        app.styleHints().colorSchemeChanged.connect(lambda *_: callback())
    except AttributeError:
        pass
