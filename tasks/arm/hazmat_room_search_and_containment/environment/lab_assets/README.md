# Laboratory assets from Poly Haven (MuJoCo-ready, textured)

**Source:** Poly Haven, https://polyhaven.com (models and textures by the Poly Haven team).
**License:** CC0 1.0 (public domain dedication) for every file here. Credit is appreciated but not required.

## What was done

- Each model is the 1k glTF from Poly Haven. It is split into one mesh per material, `<model>/<k>.obj`:
  triangles with texture coordinates (`v`, `vt`, `f v/vt`), vertices in metres in the output frame.
  `<model>/<k>.png` is that material's base-colour texture (times its base-colour factor), resampled to
  512 x 512 for furniture and 256 x 256 for boxes, bins, standing pieces and small things. A material
  without a texture gets a 4 x 4 PNG of its colour.
- Frame: z up, metres, origin at the centre of the bounding box in x and y and at its bottom (min z = 0),
  the back (wall side) on -y and the front on +y. glTF (x, y, z) -> (-x, z, y). Fronts were checked on
  rendered contact sheets.
- Scale: Poly Haven's real-world scale is kept; sizes match the sizes Poly Haven lists, except
  `steel_frame_shelves_01` (its 1k glTF is 10 times too large; scaled by 0.1) and a few rigged models whose
  listed size includes their rig or a pose (`alarm_clock_01`, `desk_lamp_arm_01`, `metal_toolbox`,
  `industrial_plastic_container`).
- Parts with more than about 6,000 triangles were decimated with pymeshlab's quadric edge collapse with
  texture (UVs kept); small things, boxes, bins and standing pieces to about 4,000 triangles per model.
- `rgba` in the manifest is the mean colour of the part's PNG over the texels its UVs cover (alpha-masked
  texels excluded): the colour a ray-cast camera should show for that part.
- `surfaces`: for desks and sideboards, axis-aligned rectangles on the top level, as seen from above
  (top-down heightmap, 5 mm cells); `z` is the highest point of the rectangle. Empty otherwise.
- Part flags: `transparent` (a glass part in the source).
- Kinds: `desk` (benches and tables), `chair` (stools and chairs), `shelves`, `sideboard` (a tool cart),
  `box`, `bin` (waste canisters), `standing` (pieces that stand along a wall), `thing` (what stands on tops).
- Surface textures (`textures` in the manifest): 512 x 512 tileable base-colour PNGs; `tile_m` is the size
  one tile is laid at. The two floor textures are laid at a lab's usual tile sizes (30 cm vinyl squares for
  `floor_tiles_06`, 60 cm stone tiles for `floor_tiles_02`) instead of Poly Haven's listed size.

## Models

| model | kind | Poly Haven asset | size x / y / z (m) | parts | triangles | top (m) | changes |
|---|---|---|---|---|---|---|---|
| `canister_5_litre` | bin | [Multi Cleaner 5 Litre](https://polyhaven.com/a/multi_cleaner_5_litre) (`multi_cleaner_5_litre`) | 0.22 x 0.11 x 0.30 | 1 | 4000 | - | converted, decimated 4332 -> 4000 tris |
| `plastic_jerrycan` | bin | [Plastic Jerrycan](https://polyhaven.com/a/plastic_jerrycan) (`plastic_jerrycan`) | 0.27 x 0.34 x 0.32 | 1 | 4000 | - | converted, decimated 7348 -> 4000 tris |
| `cardboard_box_01` | box | [Cardboard Box 01](https://polyhaven.com/a/cardboard_box_01) (`cardboard_box_01`) | 0.39 x 0.52 x 0.34 | 1 | 4000 | - | converted, decimated 16952 -> 4000 tris |
| `industrial_plastic_container` | box | [Industrial Plastic Container](https://polyhaven.com/a/industrial_pastic_container) (`industrial_pastic_container`) | 0.46 x 0.63 x 0.42 | 1 | 4000 | - | converted, decimated 7118 -> 4000 tris |
| `plastic_container` | box | [Plastic Container](https://polyhaven.com/a/plastic_container) (`plastic_container`) | 0.90 x 0.63 x 0.43 | 1 | 4000 | - | converted, decimated 4070 -> 4000 tris |
| `plastic_crate_01` | box | [Plastic Crate 01](https://polyhaven.com/a/plastic_crate_01) (`plastic_crate_01`) | 0.30 x 0.41 x 0.26 | 1 | 4000 | - | converted, decimated 18320 -> 4000 tris |
| `plastic_crate_02` | box | [Plastic Crate 02](https://polyhaven.com/a/plastic_crate_02) (`plastic_crate_02`) | 0.51 x 0.41 x 0.25 | 1 | 4000 | - | converted, decimated 5840 -> 4000 tris |
| `bar_chair_round_01` | chair | [Bar Chair Round 01](https://polyhaven.com/a/bar_chair_round_01) (`bar_chair_round_01`) | 0.48 x 0.49 x 0.75 | 1 | 6000 | - | converted, decimated 14373 -> 6000 tris |
| `metal_stool_01` | chair | [Metal Stool 01](https://polyhaven.com/a/metal_stool_01) (`metal_stool_01`) | 0.35 x 0.35 x 0.88 | 1 | 6000 | - | converted, decimated 8334 -> 6000 tris |
| `metal_stool_03` | chair | [Metal Stool 03](https://polyhaven.com/a/metal_stool_03) (`metal_stool_03`) | 0.46 x 0.47 x 1.02 | 1 | 6000 | - | converted, decimated 6576 -> 6000 tris |
| `school_chair_01` | chair | [School Chair 01](https://polyhaven.com/a/SchoolChair_01) (`SchoolChair_01`) | 0.57 x 0.68 x 1.01 | 1 | 5072 | - | converted |
| `metal_office_desk` | desk | [Metal Office Desk](https://polyhaven.com/a/metal_office_desk) (`metal_office_desk`) | 2.00 x 0.95 x 0.79 | 1 | 6000 | 0.79 | converted, decimated 6898 -> 6000 tris |
| `school_desk_01` | desk | [School Desk 01](https://polyhaven.com/a/SchoolDesk_01) (`SchoolDesk_01`) | 0.71 x 0.55 x 0.88 | 1 | 4158 | 0.88 | converted, decimated 4162 -> 4158 tris |
| `wooden_table_02` | desk | [Wooden Table 02](https://polyhaven.com/a/wooden_table_02) (`wooden_table_02`) | 1.13 x 0.71 x 0.80 | 1 | 196 | 0.80 | converted |
| `steel_frame_shelves_01` | shelves | [Steel Frame Shelves 01](https://polyhaven.com/a/steel_frame_shelves_01) (`steel_frame_shelves_01`) | 1.10 x 0.50 x 2.14 | 1 | 4348 | - | converted, scaled by 0.1 |
| `steel_frame_shelves_02` | shelves | [Steel Frame Shelves 02](https://polyhaven.com/a/steel_frame_shelves_02) (`steel_frame_shelves_02`) | 0.59 x 0.50 x 2.14 | 1 | 4348 | - | converted |
| `worn_metal_rack` | shelves | [Worn Metal Rack](https://polyhaven.com/a/worn_metal_rack) (`worn_metal_rack`) | 0.92 x 0.60 x 1.90 | 1 | 6000 | - | converted, decimated 6372 -> 6000 tris |
| `tool_cart` | sideboard | [Tool Cart](https://polyhaven.com/a/tool_cart) (`tool_cart`) | 1.27 x 0.75 x 0.96 | 1 | 6000 | 0.90 | converted, decimated 29394 -> 6000 tris |
| `fire_extinguisher_01` | standing | [Korean Fire Extinguisher 01](https://polyhaven.com/a/korean_fire_extinguisher_01) (`korean_fire_extinguisher_01`) | 0.28 x 0.37 x 0.66 | 3 | 4189 | - | converted, decimated 9913 -> 4189 tris |
| `hand_truck` | standing | [Hand Truck](https://polyhaven.com/a/hand_truck) (`hand_truck`) | 0.59 x 0.69 x 1.40 | 1 | 4000 | - | converted, decimated 13220 -> 4000 tris |
| `metal_tool_chest` | standing | [Metal Tool Chest](https://polyhaven.com/a/metal_tool_chest) (`metal_tool_chest`) | 0.69 x 0.41 x 0.65 | 1 | 4000 | - | converted, decimated 13360 -> 4000 tris |
| `propane_tank` | standing | [Propane Tank](https://polyhaven.com/a/propane_tank) (`propane_tank`) | 0.34 x 0.34 x 0.55 | 1 | 4000 | - | converted, decimated 5238 -> 4000 tris |
| `small_lpg_tank` | standing | [Small Lpg Tank](https://polyhaven.com/a/small_lpg_tank) (`small_lpg_tank`) | 0.41 x 0.41 x 0.64 | 1 | 4000 | - | converted, decimated 15042 -> 4000 tris |
| `wet_floor_sign_01` | standing | [Wet Floor Sign 01](https://polyhaven.com/a/WetFloorSign_01) (`WetFloorSign_01`) | 0.30 x 0.36 x 0.63 | 1 | 228 | - | converted |
| `alarm_clock_01` | thing | [Alarm Clock 01](https://polyhaven.com/a/alarm_clock_01) (`alarm_clock_01`) | 0.13 x 0.07 x 0.17 | 2 | 3999 | - | converted, decimated 8985 -> 3999 tris |
| `all_purpose_cleaner` | thing | [All Purpose Cleaner](https://polyhaven.com/a/all_purpose_cleaner) (`all_purpose_cleaner`) | 0.16 x 0.12 x 0.32 | 1 | 4000 | - | converted, decimated 4624 -> 4000 tris |
| `bleach_bottle` | thing | [Bleach Bottle](https://polyhaven.com/a/bleach_bottle) (`bleach_bottle`) | 0.09 x 0.06 x 0.29 | 1 | 4000 | - | converted, decimated 4052 -> 4000 tris |
| `book_encyclopedia_set_01` | thing | [Book Encyclopedia Set 01](https://polyhaven.com/a/book_encyclopedia_set_01) (`book_encyclopedia_set_01`) | 0.55 x 0.16 x 0.24 | 2 | 3999 | - | converted, decimated 67306 -> 3999 tris |
| `bunsen_burner` | thing | [Bunsen Burner](https://polyhaven.com/a/bunsen_burner) (`bunsen_burner`) | 0.14 x 0.14 x 0.17 | 1 | 4000 | - | converted, decimated 8434 -> 4000 tris |
| `chemistry_set` | thing | [Chemistry Set](https://polyhaven.com/a/chemistry_set) (`chemistry_set`) | 1.08 x 0.35 x 0.43 | 2 | 3998 | - | converted, decimated 41958 -> 3998 tris |
| `circuit_board` | thing | [Circuit Board](https://polyhaven.com/a/circuit_board) (`circuit_board`) | 0.40 x 0.40 x 0.08 | 2 | 3999 | - | converted, decimated 14430 -> 3999 tris |
| `classic_laptop` | thing | [Classic Laptop](https://polyhaven.com/a/classic_laptop) (`classic_laptop`) | 0.65 x 0.49 x 0.54 | 2 | 4017 | - | converted, decimated 14450 -> 4017 tris |
| `cleaner_bottle` | thing | [Multi Cleaner Bottle](https://polyhaven.com/a/multi_cleaner_bottle) (`multi_cleaner_bottle`) | 0.09 x 0.09 x 0.23 | 1 | 4000 | - | converted, decimated 6792 -> 4000 tris |
| `clipboard` | thing | [Clipboard](https://polyhaven.com/a/clipboard) (`clipboard`) | 0.23 x 0.34 x 0.04 | 1 | 4000 | - | converted, decimated 6176 -> 4000 tris |
| `desk_lamp_arm_01` | thing | [Desk Lamp Arm 01](https://polyhaven.com/a/desk_lamp_arm_01) (`desk_lamp_arm_01`) | 0.20 x 0.61 x 0.89 | 2 | 4050 | - | converted, decimated 24102 -> 4050 tris |
| `first_aid_box` | thing | [Medical Box](https://polyhaven.com/a/medical_box) (`medical_box`) | 0.53 x 0.35 x 0.10 | 1 | 3718 | - | converted |
| `industrial_microscope` | thing | [Industrial Microscope](https://polyhaven.com/a/industrial_microscope) (`industrial_microscope`) | 0.21 x 0.50 x 0.46 | 1 | 4000 | - | converted, decimated 16598 -> 4000 tris |
| `magnifying_glass_01` | thing | [Magnifying Glass 01](https://polyhaven.com/a/magnifying_glass_01) (`magnifying_glass_01`) | 0.13 x 0.03 x 0.27 | 2 | 3998 | - | converted, decimated 7240 -> 3998 tris |
| `metal_toolbox` | thing | [Metal Toolbox](https://polyhaven.com/a/metal_toolbox) (`metal_toolbox`) | 0.40 x 0.27 x 0.30 | 1 | 4000 | - | converted, decimated 14228 -> 4000 tris |
| `plastic_bottle_gallon` | thing | [Plastic Bottle Gallon](https://polyhaven.com/a/plastic_bottle_gallon) (`plastic_bottle_gallon`) | 0.17 x 0.13 x 0.27 | 1 | 4000 | - | converted, decimated 13728 -> 4000 tris |
| `stationery_supplies` | thing | [Stationery Supplies](https://polyhaven.com/a/stationery_supplies) (`stationery_supplies`) | 0.30 x 0.08 x 0.16 | 1 | 4000 | - | converted, decimated 4412 -> 4000 tris |
| `vintage_microscope` | thing | [Vintage Microscope](https://polyhaven.com/a/vintage_microscope) (`vintage_microscope`) | 0.11 x 0.18 x 0.40 | 2 | 4237 | - | converted, decimated 20631 -> 4237 tris |

## Surface textures

| role | file | Poly Haven texture | laid at (m per tile) |
|---|---|---|---|
| floor | `textures/floor_tiles_06.png` | [Floor Tiles 06](https://polyhaven.com/a/floor_tiles_06) (`floor_tiles_06`) | 3.0 |
| floor | `textures/floor_tiles_02.png` | [Floor Tiles 02](https://polyhaven.com/a/floor_tiles_02) (`floor_tiles_02`) | 4.0 |
| wall | `textures/painted_plaster_wall.png` | [Painted Plaster Wall](https://polyhaven.com/a/painted_plaster_wall) (`painted_plaster_wall`) | 2.0 |
| wall | `textures/concrete_wall_001.png` | [Concrete Wall 001](https://polyhaven.com/a/concrete_wall_001) (`concrete_wall_001`) | 1.5 |
| casework | `textures/grey_oak_veneer_01.png` | [Grey Oak Veneer 01](https://polyhaven.com/a/grey_oak_veneer_01) (`grey_oak_veneer_01`) | 1.0 |
| casework | `textures/blue_metal_plate.png` | [Blue Metal Plate](https://polyhaven.com/a/blue_metal_plate) (`blue_metal_plate`) | 2.5 |
| worktop | `textures/black_painted_planks.png` | [Black Painted Planks](https://polyhaven.com/a/black_painted_planks) (`black_painted_planks`) | 1.6 |
| worktop | `textures/granite_tile_04.png` | [Granite Tile 04](https://polyhaven.com/a/granite_tile_04) (`granite_tile_04`) | 2.0 |
