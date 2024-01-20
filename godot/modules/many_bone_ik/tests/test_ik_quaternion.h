/**************************************************************************/
/*  test_qcp.h                                                            */
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

#ifndef TEST_IK_QUATERNION_H
#define TEST_IK_QUATERNION_H

#include "core/io/json.h"
#include "core/math/quaternion.h"
#include "core/math/random_number_generator.h"
#include "core/string/print_string.h"
#include "core/templates/local_vector.h"
#include "modules/many_bone_ik/src/math/qcp.h"
#include "modules/many_bone_ik/src/ik_kusudama_3d.h"
#include "tests/test_macros.h"

namespace TestIKQuaternion {

// TEST_CASE("[Modules][ManyBoneIK] Match EWBIK") {
// 	LocalVector<Quaternion> randomQ_array;
// 	randomQ_array.resize(100);
// 	Ref<RandomNumberGenerator> rng;
// 	rng.instantiate();
// 	rng->set_seed(420);
// 	for (int i = 0; i < 100; i++) {
// 		Quaternion random_quaternion = Quaternion(rng->randf_range(-1.0, 1.0), rng->randf_range(-1.0, 1.0), rng->randf_range(-1.0, 1.0), rng->randf_range(-1.0, 1.0)).normalized();
// 		randomQ_array[i] = random_quaternion;
// 		CHECK(random_quaternion != Quaternion());
// 	}
// }

auto quaternion_operator_q1q2 = [](const Quaternion &q1, const Quaternion &q2) -> Quaternion {
	return (q1 * q2).normalized();
};
auto quaternion_operator_negq1q2 = [](const Quaternion &q1, const Quaternion &q2) -> Quaternion {
	return (q1.inverse() * q2).normalized();
};

auto quaternion_operator_q2q1 = [](const Quaternion &q1, const Quaternion &q2) -> Quaternion {
	return (q2 * q1).normalized();
};

auto quaternion_operator_negq2q1 = [](const Quaternion &q1, const Quaternion &q2) -> Quaternion {
	return (q2.inverse() * q1).normalized();
};

auto quaternion_operator_negq2_negq1 = [](const Quaternion &q1, const Quaternion &q2) -> Quaternion {
	return (q2.inverse() * q1.inverse()).normalized();
};

auto quaternion_operator_negq1_negq2 = [](const Quaternion &q1, const Quaternion &q2) -> Quaternion {
	return (q2.inverse() * q1.inverse()).normalized();
};

auto quaternion_operator_applyTo = [](const Quaternion &q1, const Quaternion &q2) -> Quaternion {
	return (IKKusudama3D::applyTo(q1, q2)).normalized();
};

TEST_CASE("[Modules][ManyBoneIK] Match EWBIK") {
	LocalVector<Quaternion> randomQ_array;
	randomQ_array.push_back(Quaternion(-0.07721299833, -0.5756266594, -0.5756266594, -0.5756266594));
	randomQ_array.push_back(Quaternion(-0.02554013746, -0.5771619361, -0.5771619361, -0.5771619361));
	randomQ_array.push_back(Quaternion(-0.4200069598, -0.5239574263, -0.5239574263, -0.5239574263));
	randomQ_array.push_back(Quaternion(-0.1431078754, -0.5714076583, -0.5714076583, -0.5714076583));
	randomQ_array.push_back(Quaternion(-0.2922278371, -0.5521481961, -0.5521481961, -0.5521481961));
	randomQ_array.push_back(Quaternion(-0.4499755364, -0.5155973935, -0.5155973935, -0.5155973935));
	randomQ_array.push_back(Quaternion(-0.2745564021, -0.5551632739, -0.5551632739, -0.5551632739));
	randomQ_array.push_back(Quaternion(0.4701028666, -0.509576064, -0.509576064, -0.509576064));
	randomQ_array.push_back(Quaternion(-0.09052816548, -0.5749796087, -0.5749796087, -0.5749796087));
	randomQ_array.push_back(Quaternion(-0.2517979471, -0.5587479437, -0.5587479437, -0.5587479437));
	randomQ_array.push_back(Quaternion(-0.1550604574, -0.5703672076, -0.5703672076, -0.5703672076));
	randomQ_array.push_back(Quaternion(-0.1761356171, -0.568323923, -0.568323923, -0.568323923));
	randomQ_array.push_back(Quaternion(-0.2058766606, -0.5649822418, -0.5649822418, -0.5649822418));
	randomQ_array.push_back(Quaternion(-0.2657521192, -0.5565895589, -0.5565895589, -0.5565895589));
	randomQ_array.push_back(Quaternion(0.2136471988, -0.5640197616, -0.5640197616, -0.5640197616));
	randomQ_array.push_back(Quaternion(-0.2447632462, -0.5597889344, -0.5597889344, -0.5597889344));
	randomQ_array.push_back(Quaternion(0.4698529725, -0.5096528833, -0.5096528833, -0.5096528833));
	randomQ_array.push_back(Quaternion(0.4963852316, -0.5011991294, -0.5011991294, -0.5011991294));
	randomQ_array.push_back(Quaternion(-0.4007821872, -0.5289529401, -0.5289529401, -0.5289529401));
	randomQ_array.push_back(Quaternion(0.3353346281, -0.5439211607, -0.5439211607, -0.5439211607));
	randomQ_array.push_back(Quaternion(0.1869101495, -0.567175633, -0.567175633, -0.567175633));
	randomQ_array.push_back(Quaternion(0.4344038593, -0.5200298988, -0.5200298988, -0.5200298988));
	randomQ_array.push_back(Quaternion(-0.2660351648, -0.5565444849, -0.5565444849, -0.5565444849));
	randomQ_array.push_back(Quaternion(-0.4502721595, -0.5155110676, -0.5155110676, -0.5155110676));
	randomQ_array.push_back(Quaternion(0.4601922559, -0.5125827047, -0.5125827047, -0.5125827047));
	randomQ_array.push_back(Quaternion(0.492291962, -0.5025430742, -0.5025430742, -0.5025430742));
	randomQ_array.push_back(Quaternion(-0.03828260557, -0.5769270439, -0.5769270439, -0.5769270439));
	randomQ_array.push_back(Quaternion(0.1319418411, -0.5723027609, -0.5723027609, -0.5723027609));
	randomQ_array.push_back(Quaternion(-0.09059899578, -0.5749758899, -0.5749758899, -0.5749758899));
	randomQ_array.push_back(Quaternion(-0.2184454694, -0.5634067734, -0.5634067734, -0.5634067734));
	randomQ_array.push_back(Quaternion(-0.4767093831, -0.5075260795, -0.5075260795, -0.5075260795));
	randomQ_array.push_back(Quaternion(0.04120438158, -0.576859948, -0.576859948, -0.576859948));
	randomQ_array.push_back(Quaternion(0.2292762992, -0.5619704555, -0.5619704555, -0.5619704555));
	randomQ_array.push_back(Quaternion(0.4304957682, -0.521112078, -0.521112078, -0.521112078));
	randomQ_array.push_back(Quaternion(0.1114915921, -0.573750708, -0.573750708, -0.573750708));
	randomQ_array.push_back(Quaternion(-0.2995258975, -0.550843062, -0.550843062, -0.550843062));
	randomQ_array.push_back(Quaternion(-0.1315884715, -0.5723298799, -0.5723298799, -0.5723298799));
	randomQ_array.push_back(Quaternion(-0.2546023074, -0.5583241786, -0.5583241786, -0.5583241786));
	randomQ_array.push_back(Quaternion(-0.1337230793, -0.5721649349, -0.5721649349, -0.5721649349));
	randomQ_array.push_back(Quaternion(-0.3558418688, -0.5395604892, -0.5395604892, -0.5395604892));
	randomQ_array.push_back(Quaternion(-0.3668603461, -0.5370951146, -0.5370951146, -0.5370951146));
	randomQ_array.push_back(Quaternion(0.4475400621, -0.5163034937, -0.5163034937, -0.5163034937));
	randomQ_array.push_back(Quaternion(-0.2454851448, -0.5596835545, -0.5596835545, -0.5596835545));
	randomQ_array.push_back(Quaternion(0.4837703585, -0.5052940531, -0.5052940531, -0.5052940531));
	randomQ_array.push_back(Quaternion(-0.2816568052, -0.5539763666, -0.5539763666, -0.5539763666));
	randomQ_array.push_back(Quaternion(0.4418823938, -0.5179253325, -0.5179253325, -0.5179253325));
	randomQ_array.push_back(Quaternion(-0.4912341678, -0.5028879903, -0.5028879903, -0.5028879903));
	randomQ_array.push_back(Quaternion(-0.3870240018, -0.5323571552, -0.5323571552, -0.5323571552));
	randomQ_array.push_back(Quaternion(-0.236825985, -0.5609258575, -0.5609258575, -0.5609258575));
	randomQ_array.push_back(Quaternion(0.36492518, -0.537534375, -0.537534375, -0.537534375));
	randomQ_array.push_back(Quaternion(0.2426788359, -0.5600913564, -0.5600913564, -0.5600913564));
	randomQ_array.push_back(Quaternion(-0.03793642094, -0.5769346664, -0.5769346664, -0.5769346664));
	randomQ_array.push_back(Quaternion(0.1765000958, -0.5682862296, -0.5682862296, -0.5682862296));
	randomQ_array.push_back(Quaternion(0.02947153657, -0.5770994797, -0.5770994797, -0.5770994797));
	randomQ_array.push_back(Quaternion(-0.04866040831, -0.5766663289, -0.5766663289, -0.5766663289));
	randomQ_array.push_back(Quaternion(-0.4351436944, -0.5198236769, -0.5198236769, -0.5198236769));
	randomQ_array.push_back(Quaternion(0.45960987, -0.5127568519, -0.5127568519, -0.5127568519));
	randomQ_array.push_back(Quaternion(-0.2235989495, -0.5627324734, -0.5627324734, -0.5627324734));
	randomQ_array.push_back(Quaternion(0.4532218789, -0.5146487244, -0.5146487244, -0.5146487244));
	randomQ_array.push_back(Quaternion(-0.4595609816, -0.5127714579, -0.5127714579, -0.5127714579));
	randomQ_array.push_back(Quaternion(-0.4825585232, -0.5056801597, -0.5056801597, -0.5056801597));
	randomQ_array.push_back(Quaternion(-0.318422932, -0.5472984671, -0.5472984671, -0.5472984671));
	randomQ_array.push_back(Quaternion(-0.422822965, -0.5232019178, -0.5232019178, -0.5232019178));
	randomQ_array.push_back(Quaternion(0.03857760505, -0.5769204938, -0.5769204938, -0.5769204938));
	randomQ_array.push_back(Quaternion(0.1455265055, -0.5712040021, -0.5712040021, -0.5712040021));
	randomQ_array.push_back(Quaternion(-0.3942207487, -0.5305940072, -0.5305940072, -0.5305940072));
	randomQ_array.push_back(Quaternion(-0.2436769029, -0.5599468924, -0.5599468924, -0.5599468924));
	randomQ_array.push_back(Quaternion(-0.4963404956, -0.5012138973, -0.5012138973, -0.5012138973));
	randomQ_array.push_back(Quaternion(0.1819943706, -0.5677082728, -0.5677082728, -0.5677082728));
	randomQ_array.push_back(Quaternion(0.2543796988, -0.5583580001, -0.5583580001, -0.5583580001));
	randomQ_array.push_back(Quaternion(-0.4315492467, -0.5208215458, -0.5208215458, -0.5208215458));
	randomQ_array.push_back(Quaternion(0.3578235872, -0.5391234492, -0.5391234492, -0.5391234492));
	randomQ_array.push_back(Quaternion(-0.299517738, -0.5508445409, -0.5508445409, -0.5508445409));
	randomQ_array.push_back(Quaternion(-0.2870800826, -0.5530476249, -0.5530476249, -0.5530476249));
	randomQ_array.push_back(Quaternion(-0.4029578407, -0.5284016713, -0.5284016713, -0.5284016713));
	randomQ_array.push_back(Quaternion(-0.05318362577, -0.5765331739, -0.5765331739, -0.5765331739));
	randomQ_array.push_back(Quaternion(0.2583565569, -0.5577490145, -0.5577490145, -0.5577490145));
	randomQ_array.push_back(Quaternion(-0.4225827808, -0.5232665966, -0.5232665966, -0.5232665966));
	randomQ_array.push_back(Quaternion(-0.221531256, -0.5630050037, -0.5630050037, -0.5630050037));
	randomQ_array.push_back(Quaternion(0.4950693441, -0.5016327822, -0.5016327822, -0.5016327822));
	randomQ_array.push_back(Quaternion(0.2314502044, -0.5616733341, -0.5616733341, -0.5616733341));
	randomQ_array.push_back(Quaternion(0.4428563264, -0.5176479737, -0.5176479737, -0.5176479737));
	randomQ_array.push_back(Quaternion(-0.4619823523, -0.5120456706, -0.5120456706, -0.5120456706));
	randomQ_array.push_back(Quaternion(-0.1231485312, -0.5729556234, -0.5729556234, -0.5729556234));
	randomQ_array.push_back(Quaternion(-0.1700987364, -0.5689365576, -0.5689365576, -0.5689365576));
	randomQ_array.push_back(Quaternion(0.1822882855, -0.5676768391, -0.5676768391, -0.5676768391));
	randomQ_array.push_back(Quaternion(0.3618855681, -0.5382189256, -0.5382189256, -0.5382189256));
	randomQ_array.push_back(Quaternion(0.4770307958, -0.5074254033, -0.5074254033, -0.5074254033));
	randomQ_array.push_back(Quaternion(-0.06152142602, -0.5762566309, -0.5762566309, -0.5762566309));
	randomQ_array.push_back(Quaternion(0.2245170177, -0.5626106139, -0.5626106139, -0.5626106139));
	randomQ_array.push_back(Quaternion(0.4949252505, -0.5016801758, -0.5016801758, -0.5016801758));
	randomQ_array.push_back(Quaternion(-0.1074310665, -0.5740088751, -0.5740088751, -0.5740088751));
	randomQ_array.push_back(Quaternion(0.2721395065, -0.5555598045, -0.5555598045, -0.5555598045));
	randomQ_array.push_back(Quaternion(-0.0355963391, -0.5769843732, -0.5769843732, -0.5769843732));
	randomQ_array.push_back(Quaternion(-0.4962919392, -0.5012299243, -0.5012299243, -0.5012299243));
	randomQ_array.push_back(Quaternion(0.2163136966, -0.5636808744, -0.5636808744, -0.5636808744));
	randomQ_array.push_back(Quaternion(0.2838168882, -0.5536087589, -0.5536087589, -0.5536087589));
	randomQ_array.push_back(Quaternion(-0.249391906, -0.5591075261, -0.5591075261, -0.5591075261));
	randomQ_array.push_back(Quaternion(0.2814001766, -0.5540198374, -0.5540198374, -0.5540198374));
	randomQ_array.push_back(Quaternion(-0.3283354743, -0.5453426496, -0.5453426496, -0.5453426496));

	LocalVector<Pair<int, int>> quaternion_pairs;
	quaternion_pairs.push_back(Pair<int, int>(11, 98));
	quaternion_pairs.push_back(Pair<int, int>(50, 6));
	quaternion_pairs.push_back(Pair<int, int>(10, 54));
	quaternion_pairs.push_back(Pair<int, int>(2, 58));
	quaternion_pairs.push_back(Pair<int, int>(44, 98));
	quaternion_pairs.push_back(Pair<int, int>(24, 12));
	quaternion_pairs.push_back(Pair<int, int>(17, 10));
	quaternion_pairs.push_back(Pair<int, int>(82, 74));
	quaternion_pairs.push_back(Pair<int, int>(42, 29));
	quaternion_pairs.push_back(Pair<int, int>(42, 93));
	quaternion_pairs.push_back(Pair<int, int>(61, 84));
	quaternion_pairs.push_back(Pair<int, int>(50, 42));
	quaternion_pairs.push_back(Pair<int, int>(23, 54));
	quaternion_pairs.push_back(Pair<int, int>(98, 40));
	quaternion_pairs.push_back(Pair<int, int>(58, 4));
	quaternion_pairs.push_back(Pair<int, int>(2, 84));
	quaternion_pairs.push_back(Pair<int, int>(46, 77));
	quaternion_pairs.push_back(Pair<int, int>(71, 2));
	quaternion_pairs.push_back(Pair<int, int>(89, 99));
	quaternion_pairs.push_back(Pair<int, int>(16, 56));
	quaternion_pairs.push_back(Pair<int, int>(82, 43));
	quaternion_pairs.push_back(Pair<int, int>(1, 11));
	quaternion_pairs.push_back(Pair<int, int>(20, 72));
	quaternion_pairs.push_back(Pair<int, int>(61, 85));
	quaternion_pairs.push_back(Pair<int, int>(35, 0));
	quaternion_pairs.push_back(Pair<int, int>(79, 10));
	quaternion_pairs.push_back(Pair<int, int>(16, 95));
	quaternion_pairs.push_back(Pair<int, int>(43, 43));
	quaternion_pairs.push_back(Pair<int, int>(97, 59));
	quaternion_pairs.push_back(Pair<int, int>(29, 55));
	quaternion_pairs.push_back(Pair<int, int>(99, 45));
	quaternion_pairs.push_back(Pair<int, int>(23, 24));
	quaternion_pairs.push_back(Pair<int, int>(28, 43));
	quaternion_pairs.push_back(Pair<int, int>(95, 59));
	quaternion_pairs.push_back(Pair<int, int>(82, 99));
	quaternion_pairs.push_back(Pair<int, int>(35, 38));
	quaternion_pairs.push_back(Pair<int, int>(57, 0));
	quaternion_pairs.push_back(Pair<int, int>(63, 81));
	quaternion_pairs.push_back(Pair<int, int>(31, 51));
	quaternion_pairs.push_back(Pair<int, int>(0, 36));
	quaternion_pairs.push_back(Pair<int, int>(18, 62));
	quaternion_pairs.push_back(Pair<int, int>(1, 29));
	quaternion_pairs.push_back(Pair<int, int>(26, 5));
	quaternion_pairs.push_back(Pair<int, int>(77, 35));
	quaternion_pairs.push_back(Pair<int, int>(78, 76));
	quaternion_pairs.push_back(Pair<int, int>(7, 26));
	quaternion_pairs.push_back(Pair<int, int>(16, 25));
	quaternion_pairs.push_back(Pair<int, int>(28, 0));
	quaternion_pairs.push_back(Pair<int, int>(39, 60));
	quaternion_pairs.push_back(Pair<int, int>(46, 66));
	quaternion_pairs.push_back(Pair<int, int>(17, 55));
	quaternion_pairs.push_back(Pair<int, int>(46, 36));
	quaternion_pairs.push_back(Pair<int, int>(53, 83));
	quaternion_pairs.push_back(Pair<int, int>(12, 80));
	quaternion_pairs.push_back(Pair<int, int>(82, 31));
	quaternion_pairs.push_back(Pair<int, int>(29, 49));
	quaternion_pairs.push_back(Pair<int, int>(33, 35));
	quaternion_pairs.push_back(Pair<int, int>(27, 69));
	quaternion_pairs.push_back(Pair<int, int>(53, 34));
	quaternion_pairs.push_back(Pair<int, int>(33, 39));
	quaternion_pairs.push_back(Pair<int, int>(62, 49));
	quaternion_pairs.push_back(Pair<int, int>(29, 31));
	quaternion_pairs.push_back(Pair<int, int>(94, 76));
	quaternion_pairs.push_back(Pair<int, int>(78, 19));
	quaternion_pairs.push_back(Pair<int, int>(48, 86));
	quaternion_pairs.push_back(Pair<int, int>(8, 68));
	quaternion_pairs.push_back(Pair<int, int>(82, 25));
	quaternion_pairs.push_back(Pair<int, int>(46, 82));
	quaternion_pairs.push_back(Pair<int, int>(70, 74));
	quaternion_pairs.push_back(Pair<int, int>(80, 26));
	quaternion_pairs.push_back(Pair<int, int>(30, 16));
	quaternion_pairs.push_back(Pair<int, int>(71, 35));
	quaternion_pairs.push_back(Pair<int, int>(47, 65));
	quaternion_pairs.push_back(Pair<int, int>(85, 17));
	quaternion_pairs.push_back(Pair<int, int>(81, 50));
	quaternion_pairs.push_back(Pair<int, int>(88, 47));
	quaternion_pairs.push_back(Pair<int, int>(6, 79));
	quaternion_pairs.push_back(Pair<int, int>(29, 17));
	quaternion_pairs.push_back(Pair<int, int>(25, 83));
	quaternion_pairs.push_back(Pair<int, int>(64, 36));
	quaternion_pairs.push_back(Pair<int, int>(58, 43));
	quaternion_pairs.push_back(Pair<int, int>(35, 56));
	quaternion_pairs.push_back(Pair<int, int>(27, 4));
	quaternion_pairs.push_back(Pair<int, int>(68, 75));
	quaternion_pairs.push_back(Pair<int, int>(15, 49));
	quaternion_pairs.push_back(Pair<int, int>(36, 87));
	quaternion_pairs.push_back(Pair<int, int>(36, 30));
	quaternion_pairs.push_back(Pair<int, int>(61, 24));
	quaternion_pairs.push_back(Pair<int, int>(15, 78));
	quaternion_pairs.push_back(Pair<int, int>(83, 49));
	quaternion_pairs.push_back(Pair<int, int>(1, 62));
	quaternion_pairs.push_back(Pair<int, int>(71, 83));
	quaternion_pairs.push_back(Pair<int, int>(13, 51));
	quaternion_pairs.push_back(Pair<int, int>(65, 1));
	quaternion_pairs.push_back(Pair<int, int>(82, 89));
	quaternion_pairs.push_back(Pair<int, int>(2, 71));
	quaternion_pairs.push_back(Pair<int, int>(37, 93));
	quaternion_pairs.push_back(Pair<int, int>(57, 1));
	quaternion_pairs.push_back(Pair<int, int>(51, 97));
	quaternion_pairs.push_back(Pair<int, int>(92, 53));

	Vector<Vector<float>> target_quaternions = {
		{ 0.42692677423158354, -0.3053190837362794, -0.049197216495628485, -0.8497608018964722 },
		{ 0.6562566197181176, -0.56363740294201, 0.3602701414743991, 0.34906382259259244 },
		{ -0.059859534343288535, -0.09718462019533844, 0.03229413099676237, 0.9929396128913663 },
		{ 0.6598172788130158, -0.3438941893602466, -0.17590095033389455, 0.6445438703266666 },
		{ -0.12594011263249325, 0.6378005356136541, 0.16144292298964283, 0.7424861934188483 },
		{ -0.5189938344769275, -0.4822608669930415, 0.03247108998407207, 0.7049932512140111 },
		{ -0.44502008349089645, 0.336885655097169, -0.6927176603110162, 0.4567356169303401 },
		{ 0.43217051343627494, -0.7350516375172955, 0.3072885155350094, 0.42249438542974677 },
		{ 0.09390632167296638, -0.6634729504590606, -0.007037854601640325, 0.7422504397853925 },
		{ -0.2168255969999908, 0.8716148256677588, -0.07933676087863198, -0.43241176502848144 },
		{ -0.980305353721271, 0.04250946236704987, -0.002273571455619682, 0.19284498942807488 },
		{ -0.8251065594640913, -0.3969175241917564, 0.17486053424414688, -0.36204894431922924 },
		{ 0.22999758882306917, 0.3872762770995123, -0.8285090490686261, -0.33270249464486623 },
		{ -0.8618420051651322, -0.14031980199235625, -0.48635048206320813, 0.03165311830855269 },
		{ 0.6148669383004837, -0.32430651520717224, 0.3069550572553279, 0.6500327108727281 },
		{ 0.45602471883790413, 0.06768771582053483, 0.14617694837691597, 0.8752668899821481 },
		{ 0.8155995456462164, -0.1753298972183699, 0.5487389011902718, 0.05424413888683737 },
		{ -0.10873083573887991, -0.6520556529581029, 0.4554603901838306, -0.5962858909772222 },
		{ -0.3344099903392619, 0.5325087576370958, -0.5113295192341692, 0.5857870809098175 },
		{ 0.2345098571555364, 0.42005760956582655, 0.7660718717358798, 0.42625182565922154 },
		{ 0.5409941474304787, 0.2892443798254932, -0.7074994983693337, 0.3508667567503034 },
		{ 0.7449190141863583, 0.6394556873746877, -0.18482827829736423, -0.04506210968676269 },
		{ -0.6122061708189518, -0.7863539647147433, -0.009919627916954346, 0.08217449464760927 },
		{ -0.2880487129062124, 0.8337209092932558, -0.46799997669008436, -0.05397597816048995 },
		{ 0.5712702856613731, -0.2911537548665283, -0.684518673801783, -0.34686299451533165 },
		{ -0.975826149797898, 0.10528071262505473, -0.109034646956702, -0.15745076272536349 },
		{ -0.9091990985373778, -0.2244860856965519, -0.02843659070628506, -0.34950587528175153 },
		{ 0.525830587552373, 0.39672422543687524, 0.5410342814333394, -0.5228709099383401 },
		{ -0.6339551121632112, 0.11868229255676845, -0.6233527523616919, -0.4420936273219772 },
		{ 0.6515125710555854, -0.6458548386303389, 0.0016110137339270247, 0.39799535400437647 },
		{ 0.2642843536176463, 0.11680134059430601, 0.7685625934501182, -0.5708088710045635 },
		{ 0.9388545768700848, 0.010902744647469686, -0.06135158536016448, -0.33862840492646307 },
		{ -0.36362593737233595, -0.7104217018500145, 0.30752259295702483, -0.5181766475166621 },
		{ -0.638279225942129, 0.7170907825019176, -0.244491969334608, -0.1363969072684178 },
		{ -0.1724537418520431, 0.4903620966830299, -0.5896918237109513, 0.6181167156017577 },
		{ -0.7490184384735549, -0.09837834108395964, 0.6140063430271596, 0.22866851894088927 },
		{ 0.023445586576674164, 0.5384793367624382, -0.7608032775504165, -0.3614812321801844 },
		{ 0.3379284331616137, -0.2416449321244757, -0.8839826220761338, 0.21444538863518947 },
		{ -0.7233124993334171, -0.009427413727695535, -0.5556435303536238, 0.40986634328109184 },
		{ -0.2561943083832403, -0.9619409898416776, -0.06829758166354168, 0.066101805976529 },
		{ -0.04822548990937454, -0.9098616104797128, -0.09529070337525172, -0.4009312082496375 },
		{ -0.9035430009751358, -0.06580069507235999, 0.3038624070984665, 0.2948693803519887 },
		{ -0.7846832748700876, 0.5703851692503498, 0.03811439660203268, 0.2397503067986932 },
		{ 0.12443965647321115, -0.05176414732198621, 0.9866148602369683, -0.0917952205098081 },
		{ -0.907972705343376, 0.3693621263724708, -0.13723723426133957, -0.1425592069469244 },
		{ -0.5912104302769933, 0.40521920864081934, 0.4160287100050044, 0.5596317829892908 },
		{ 0.7736195331773732, -0.003877821144338209, 0.5041176057214883, -0.38387917368780194 },
		{ 0.2627291183869943, 0.6840802791097005, -0.6418272941447968, 0.22597634074290984 },
		{ 0.3121646909854631, 0.36242322139858124, -0.44760844681888873, 0.7555456919534435 },
		{ -0.44167112057009356, 0.03093123263839273, 0.22961002291339028, 0.8667462820684839 },
		{ -0.4462005374465796, 0.25540314182184715, -0.5150526093667062, -0.6858535741063542 },
		{ -0.24416728120663075, -0.14381082819833182, -0.8716813714782871, 0.39984043204609065 },
		{ -0.5444503977142281, 0.1085375374361807, 0.830669226298592, 0.042213787754799816 },
		{ -0.5600913620480246, 0.18033143773962612, -0.02180863772071555, -0.8082713789580295 },
		{ 0.4890922302987877, -0.7454493168866304, -0.21994692041625932, -0.3958755592531795 },
		{ -0.4689758102370284, -0.7870979022927163, -0.3810075757543302, -0.12398309899634252 },
		{ -0.7712618478743802, -0.2739944704389991, -0.469610024803505, 0.3309812937105363 },
		{ 0.6054138939101993, -0.3368104473111961, 0.5482318989095806, -0.46848108250015547 },
		{ 0.08481969511257961, -0.025486870228675483, 0.8749400065166193, 0.47606304599669774 },
		{ 0.6099285289572989, -0.20002464076999443, 0.43312680902721257, -0.6327546917650828 },
		{ 0.3310578925013632, -0.6784679398748873, -0.3807631084315078, 0.5339488567569539 },
		{ 0.25692371521134727, -0.6528067497219551, -0.7108951027098441, -0.04961557238020577 },
		{ -0.06997246404533261, 0.008703830754054412, -0.9193662902716361, -0.38703193914430273 },
		{ 0.4867273599398398, 0.5586926378470558, -0.021963578051456475, 0.6711755468881966 },
		{ 0.08117283333460988, -0.20319444931209393, 0.9474417907983184, 0.23340317038562722 },
		{ -0.2263230606439193, 0.12292612498541632, 0.48716540798645386, -0.8344680373017476 },
		{ 0.14762611866130754, -0.6341730821250233, -0.7569106306185106, -0.05583303908739693 },
		{ 0.37511715862202866, 0.5518041669437102, -0.027632302650587953, -0.7443357673127903 },
		{ 0.22799882976186342, 0.7543640554576099, -0.5464053295042711, -0.28353592603072014 },
		{ -0.31799947742638923, -0.7016398155827255, 0.45123180382744743, 0.4505194343868092 },
		{ -0.17663180839496437, -0.8407834236997116, 0.4684388591695626, 0.2060327010812474 },
		{ 0.7236613206706154, -0.2081054912223837, -0.07221359049186651, 0.6540577916641802 },
		{ -0.7182776933275555, 0.031417833460779726, -0.04572556579729331, -0.6935410929722869 },
		{ 0.20490450687670497, -0.3854317170923741, 0.6559673052410218, -0.6157624777263315 },
		{ 0.8519711306776431, -0.09445520373085947, 0.5047297490014029, 0.10232930886603929 },
		{ -0.29118425736347703, 0.2628933146255537, -0.5093390896575481, -0.7659455105526047 },
		{ -0.9271239633209707, -0.22413277645816276, -0.140282668982559, 0.2655681229671602 },
		{ -0.09554025968318941, 0.18989757378759747, 0.9686878982804448, -0.1282759758266034 },
		{ 0.6770013735937477, 0.2034033000410605, 0.39287439145363684, 0.5881717013122271 },
		{ -0.7283307058589981, 0.5296548466365798, -0.3948861211257228, 0.18183805344205592 },
		{ 0.628757869664127, -0.03778461087618389, 0.32782082109856636, 0.7041089218087113 },
		{ -0.19796682244565808, 0.4221160103850615, -0.8532307937185414, 0.23371868482814098 },
		{ 0.4027615487260622, -0.7173582156800918, 0.5347939358866597, 0.19281019527741258 },
		{ -0.4567235818631421, 0.2905637033259533, -0.48228544930540385, -0.6887503535156216 },
		{ -0.2858606139848019, -0.7219638583878394, -0.40655575026673113, 0.4814190674236235 },
		{ 0.48349141170529114, -0.11094186755774689, -0.8442697135959498, -0.20282161505775703 },
		{ -0.2968692833468911, 0.5340020534746117, -0.4554871062775529, -0.6474889431526243 },
		{ 0.5779625400967651, 0.326607035035245, 0.7465763947952416, 0.043713083223689275 },
		{ 0.9279609470987435, -0.05645047354320853, -0.2161022138036468, 0.29833145641301995 },
		{ -0.22999470468088654, -0.30591922405733396, 0.26003747714906433, 0.8865079664892248 },
		{ -0.2489734999053359, -0.7878255348805049, 0.050688086601916114, 0.5610470932210334 },
		{ -0.031703580418766586, 0.2947858788892749, -0.04902162580068086, -0.9537783017032662 },
		{ 0.38074078155019025, -0.7534889702567982, 0.4584694634687917, -0.27766270911402297 },
		{ -0.14372299579723205, -0.003333450463358075, -0.5848225026457561, 0.7983202546511219 },
		{ 0.0476601724196367, -0.3495163342336353, -0.23991956764725747, -0.9044365323939348 },
		{ 0.02730751458483056, 0.4725219910958785, 0.6483983372892671, -0.5962858909772222 },
		{ -0.23113727828946207, -0.5680423246364731, 0.11760146378018756, 0.7810719376109276 },
		{ -0.3267543626800189, -0.41406870942962165, -0.37490532711671215, -0.7623809323693592 },
		{ -0.2435113932005873, -0.4349770542350438, 0.648455447147094, 0.5753283381998276 },
		{ -0.45804975920291635, -0.323089376133921, 0.8239048850526615, -0.08357280364990029 }
	};

	Ref<RandomNumberGenerator> rng;
	rng.instantiate();
	rng->set_seed(420);
	Array q1q2_results;
	Array negq1q2_results;
	Array q2q1_results;
	Array negq2q1_results;
	Array negq2_negq1_results;
	Array negq1_negq2_results;
	Array applyTo_results;
	Array target_quaternions_results;

	for (int i = 0; i < 2; i++) {
		Pair<int, int> pair = quaternion_pairs[i];
		Quaternion q1q2 = quaternion_operator_q1q2(randomQ_array[pair.first], randomQ_array[pair.second]);
		Quaternion negq1q2 = quaternion_operator_negq1q2(randomQ_array[pair.first], randomQ_array[pair.second]);
		Quaternion q2q1 = quaternion_operator_q2q1(randomQ_array[pair.first], randomQ_array[pair.second]);
		Quaternion negq2q1 = quaternion_operator_negq2q1(randomQ_array[pair.first], randomQ_array[pair.second]);
		Quaternion negq2_negq1 = quaternion_operator_negq2_negq1(randomQ_array[pair.first], randomQ_array[pair.second]);
		Quaternion negq1_negq2 = quaternion_operator_negq1_negq2(randomQ_array[pair.first], randomQ_array[pair.second]);
		Quaternion applyTo_q1q2 = quaternion_operator_applyTo(randomQ_array[pair.first], randomQ_array[pair.second]);
		Vector<float> targarr = target_quaternions[i];
		Quaternion target = Quaternion(targarr[0], targarr[1], targarr[2], targarr[3]);

		q1q2_results.push_back(q1q2 - target);
		negq1q2_results.push_back(negq1q2 - target);
		q2q1_results.push_back(q2q1 - target);
		negq2q1_results.push_back(negq2q1 - target);
		negq2_negq1_results.push_back(negq2_negq1 - target);
		negq1_negq2_results.push_back(negq1_negq2 - target);
		applyTo_results.push_back(applyTo_q1q2-target);
		target_quaternions_results.push_back(target_quaternions[i]);
	}

	CHECK_MESSAGE(!quaternion_pairs.is_empty(), JSON::stringify(q1q2_results));
	CHECK_MESSAGE(!quaternion_pairs.is_empty(), JSON::stringify(negq1q2_results));
	CHECK_MESSAGE(!quaternion_pairs.is_empty(), JSON::stringify(negq2q1_results));
	CHECK_MESSAGE(!quaternion_pairs.is_empty(), JSON::stringify(negq2_negq1_results));
	CHECK_MESSAGE(!quaternion_pairs.is_empty(), JSON::stringify(negq1_negq2_results));
	CHECK_MESSAGE(!quaternion_pairs.is_empty(), JSON::stringify(applyTo_results));
	CHECK_MESSAGE(!quaternion_pairs.is_empty(), JSON::stringify(target_quaternions_results));
}

} // namespace TestIKQuaternion

#endif // TEST_IK_QUATERNION_H
