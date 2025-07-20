// Copyright (c) 2025, Koło Naukowe Robotyków
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Authors: Bartłomiej Krajewski (https://github.com/BartlomiejK2)
 */

#ifndef CONTACT_SENSORS_BROADCASTER__VISIBILITY_CONTROL_H_
#define CONTACT_SENSORS_BROADCASTER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define contact_sensors_broadcaster_EXPORT __attribute__((dllexport))
#define contact_sensors_broadcaster_IMPORT __attribute__((dllimport))
#else
#define contact_sensors_broadcaster_EXPORT __declspec(dllexport)
#define contact_sensors_broadcaster_IMPORT __declspec(dllimport)
#endif
#ifdef contact_sensors_broadcaster_BUILDING_DLL
#define contact_sensors_broadcaster_PUBLIC contact_sensors_broadcaster_EXPORT
#else
#define contact_sensors_broadcaster_PUBLIC contact_sensors_broadcaster_IMPORT
#endif
#define contact_sensors_broadcaster_PUBLIC_TYPE contact_sensors_broadcaster_PUBLIC
#define contact_sensors_broadcaster_LOCAL
#else
#define contact_sensors_broadcaster_EXPORT __attribute__((visibility("default")))
#define contact_sensors_broadcaster_IMPORT
#if __GNUC__ >= 4
#define contact_sensors_broadcaster_PUBLIC __attribute__((visibility("default")))
#define contact_sensors_broadcaster_LOCAL __attribute__((visibility("hidden")))
#else
#define contact_sensors_broadcaster_PUBLIC
#define contact_sensors_broadcaster_LOCAL
#endif
#define contact_sensors_broadcaster_PUBLIC_TYPE
#endif

#endif  // contact_sensors_broadcaster__VISIBILITY_CONTROL_H_