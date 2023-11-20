"""
Copyright (C) 2022 Adobe.
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

# file: props/utils.py
# brief: Auxiliary functions for Property Groups
# author Adobe - 3D & Immersive
# copyright 2022 Adobe Inc. All rights reserved.
# Substance3DInBlender v 1.0.2


from ..common import FORMATS_DICT, ADDON_PACKAGE


def get_shader_presets(self, context):
    _shader_presets = []
    _addon_prefs = context.preferences.addons[ADDON_PACKAGE].preferences
    for _idx,  _shader_preset in enumerate(_addon_prefs.shader_presets):
        _shader_presets.append(
            (str(_idx), _shader_preset.label, _shader_preset.filename)
        )
    return _shader_presets


def get_bitdepths(self, context, attribute):
    _bitdepths = []
    _src_format = getattr(self, attribute)
    for _bitdepth in FORMATS_DICT[_src_format]["bitdepth"]:
        _bitdepths.append(
            (_bitdepth, _bitdepth, _bitdepth)
        )
    return _bitdepths
