@tool
extends EditorPlugin

func _enter_tree() -> void:
	add_autoload_singleton("SnailTransition", "res://addons/snail_transition/snail_transition.gd")

func _exit_tree() -> void:
	remove_autoload_singleton("SnailTransition")
