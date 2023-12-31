# Copyright (c) 2018-present. This file is part of V-Sekai https://v-sekai.org/.
# SaracenOne & K. S. Ernest (Fire) Lee & Lyuma & MMMaellon & Contributors
# vsk_progress_dialog.gd
# SPDX-License-Identifier: MIT

@tool
extends Window

signal cancel_button_pressed

@export var progress_dialog_body: NodePath = NodePath()


func set_progress_bar_value(p_value: float) -> void:
	get_node(progress_dialog_body).set_progress_bar_value(p_value)


func set_progress_label_text(p_text: String) -> void:
	get_node(progress_dialog_body).set_progress_label_text(p_text)


func _on_cancel_button_pressed() -> void:
	cancel_button_pressed.emit()
