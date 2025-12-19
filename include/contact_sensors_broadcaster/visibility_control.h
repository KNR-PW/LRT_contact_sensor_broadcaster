// Copyright (c) 2025, Koło Naukowe Robotyków
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

/*
 * Authors: Bartłomiej Krajewski (https://github.com/BartlomiejK2)
 */

#ifndef CONTACT_SENSORS_BROADCASTER__VISIBILITY_CONTROL_H_
#define CONTACT_SENSORS_BROADCASTER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define CONTACT_SENSORS_BROADCASTER_EXPORT __attribute__((dllexport))
#define CONTACT_SENSORS_BROADCASTER_IMPORT __attribute__((dllimport))
#else
#define CONTACT_SENSORS_BROADCASTER_EXPORT __declspec(dllexport)
#define CONTACT_SENSORS_BROADCASTER_IMPORT __declspec(dllimport)
#endif
#ifdef CONTACT_SENSORS_BROADCASTER_BUILDING_DLL
#define CONTACT_SENSORS_BROADCASTER_PUBLIC CONTACT_SENSORS_BROADCASTER_EXPORT
#else
#define CONTACT_SENSORS_BROADCASTER_PUBLIC CONTACT_SENSORS_BROADCASTER_IMPORT
#endif
#define CONTACT_SENSORS_BROADCASTER_PUBLIC_TYPE CONTACT_SENSORS_BROADCASTER_PUBLIC
#define CONTACT_SENSORS_BROADCASTER_LOCAL
#else
#define CONTACT_SENSORS_BROADCASTER_EXPORT __attribute__((visibility("default")))
#define CONTACT_SENSORS_BROADCASTER_IMPORT
#if __GNUC__ >= 4
#define CONTACT_SENSORS_BROADCASTER_PUBLIC __attribute__((visibility("default")))
#define CONTACT_SENSORS_BROADCASTER_LOCAL __attribute__((visibility("hidden")))
#else
#define CONTACT_SENSORS_BROADCASTER_PUBLIC
#define CONTACT_SENSORS_BROADCASTER_LOCAL
#endif
#define CONTACT_SENSORS_BROADCASTER_PUBLIC_TYPE
#endif

#endif  // CONTACT_SENSORS_BROADCASTER__VISIBILITY_CONTROL_H_