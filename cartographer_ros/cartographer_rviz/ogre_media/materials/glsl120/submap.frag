/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#version 120

varying vec2 out_submap_texture_coordinate;

uniform sampler2D u_submap;
uniform float u_alpha;
uniform vec4 special_color;

void main()
{
  vec3 texture_value = texture2D(u_submap, out_submap_texture_coordinate).rgb;
  float value = u_alpha * texture_value.r;
  float alpha = u_alpha * (texture_value.g);
  gl_FragColor = vec4(value, value * (1.0 - texture_value.b), value * (1.0 - texture_value.b), alpha) * special_color;
}
