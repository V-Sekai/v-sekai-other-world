/**************************************************************************/
/*  fbx_animation.cpp                                                     */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#include "fbx_animation.h"

void FBXAnimation::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_loop"), &FBXAnimation::get_loop);
	ClassDB::bind_method(D_METHOD("set_loop", "loop"), &FBXAnimation::set_loop);

	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "loop"), "set_loop", "get_loop"); // bool
}

bool FBXAnimation::get_loop() const {
	return loop;
}

void FBXAnimation::set_loop(bool p_val) {
	loop = p_val;
}

HashMap<int, FBXAnimation::Track> &FBXAnimation::get_tracks() {
	return tracks;
}

double FBXAnimation::get_time_begin() const {
	return time_begin;
}

void FBXAnimation::set_time_begin(double p_val) {
	time_begin = p_val;
}

double FBXAnimation::get_time_end() const {
	return time_end;
}

void FBXAnimation::set_time_end(double p_val) {
	time_end = p_val;
}

HashMap<int, FBXAnimation::BlendShapeTrack> &FBXAnimation::get_blend_tracks() {
	return blend_tracks;
}

FBXAnimation::FBXAnimation() {
}
