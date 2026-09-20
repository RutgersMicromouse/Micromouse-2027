#include "decomposer.h"

Direction Decomposer::getDirection(Coordinate from, Coordinate to) {
    int8_t dx = to.x - from.x;
    int8_t dy = to.y - from.y;
    if (dx == 0 && dy == 1)  return DIR_NORTH;
    if (dx == 1 && dy == 0)  return DIR_EAST;
    if (dx == 0 && dy == -1) return DIR_SOUTH;
    if (dx == -1 && dy == 0) return DIR_WEST;
    return DIR_INVALID;
}

uint8_t Decomposer::findSlalomLength(const Direction* dirs, uint8_t start_idx, uint8_t n) {
    if (start_idx + 4 >= n) return 0;

    Direction d_prog = dirs[start_idx];
    Direction d_c1 = dirs[start_idx + 1];
    int8_t p_diff = ((int8_t)d_c1 - (int8_t)d_prog + 4) % 4;
    if (p_diff != 1 && p_diff != 3) return 0; // Must be perpendicular

    Direction d_c2 = (Direction)(((int8_t)d_c1 + 2) % 4);
    uint8_t length = 2;

    while (start_idx + length < n) {
        uint8_t rel = length;
        if (rel % 2 == 0) {
            if (dirs[start_idx + length] != d_prog) break;
        } else {
            Direction exp_c = ((rel / 2) % 2 == 0) ? d_c1 : d_c2;
            if (dirs[start_idx + length] != exp_c) break;
        }
        length++;
    }

    // Must be an odd length >= 5 so entrance & exit align with d_prog
    while (length >= 5 && length % 2 == 0) {
        length--;
    }

    return (length >= 5) ? length : 0;
}

uint8_t Decomposer::decompose(const Coordinate* path, uint8_t path_len,
                              PathSegment* out_segments, uint8_t max_segments,
                              bool allow_diagonals) {
    if (!path || path_len < 2 || !out_segments || max_segments == 0) return 0;

    static Direction dirs[256];
    uint8_t n = path_len - 1;
    for (uint8_t i = 0; i < n; ++i) {
        dirs[i] = getDirection(path[i], path[i + 1]);
    }

    uint8_t seg_count = 0;
    uint8_t i = 0;

    while (i < n && seg_count < max_segments) {
        if (allow_diagonals) {
            // 1. Up-and-Down Slalom Zigzag
            uint8_t slen = findSlalomLength(dirs, i, n);
            if (slen >= 5) {
                out_segments[seg_count++] = {
                    SEG_SLALOM,
                    dirs[i],             // Progression direction
                    dirs[i + 1],         // First wave direction
                    dirs[i],             // Exit direction aligns with prog
                    slen,
                    path[i].x, path[i].y,
                    path[i + slen].x, path[i + slen].y
                };
                i += slen;
                continue;
            }

            // Avoid bundling dirs[i] into a 2-step diagonal if the NEXT step starts a slalom
            if (i + 1 < n && findSlalomLength(dirs, i + 1, n) >= 5) {
                out_segments[seg_count++] = {
                    SEG_STRAIGHT,
                    dirs[i],
                    DIR_INVALID,
                    dirs[i],
                    1,
                    path[i].x, path[i].y,
                    path[i + 1].x, path[i + 1].y
                };
                i += 1;
                continue;
            }

            // 2. Big Diagonal Staircase (M >= 2)
            uint8_t diag_len = 0;
            if (i + 1 < n) {
                Direction d_a = dirs[i];
                Direction d_b = dirs[i + 1];
                int8_t diff = ((int8_t)d_b - (int8_t)d_a + 4) % 4;
                if (diff == 1 || diff == 3) {
                    uint8_t k = 2;
                    while (i + k < n) {
                        Direction exp = (k % 2 == 0) ? d_a : d_b;
                        if (dirs[i + k] == exp) {
                            k++;
                        } else {
                            break;
                        }
                    }
                    diag_len = k;
                }
            }

            if (diag_len >= 2) {
                out_segments[seg_count++] = {
                    SEG_DIAGONAL,
                    dirs[i],             // Entry direction d1
                    dirs[i + 1],         // Secondary cross direction d2
                    dirs[i + diag_len - 1], // Exit direction d_last
                    diag_len,
                    path[i].x, path[i].y,
                    path[i + diag_len].x, path[i + diag_len].y
                };
                i += diag_len;
                continue;
            }
        }

        // 3. Straight Sprint
        Direction d = dirs[i];
        uint8_t cnt = 1;
        while (i + cnt < n && dirs[i + cnt] == d) {
            if (allow_diagonals) {
                if (findSlalomLength(dirs, i + cnt, n) >= 5) break;
                if (i + cnt + 1 < n && findSlalomLength(dirs, i + cnt + 1, n) >= 5) break;
                if (i + cnt + 1 < n) {
                    int8_t p_diff = ((int8_t)dirs[i + cnt + 1] - (int8_t)d + 4) % 4;
                    if (p_diff == 1 || p_diff == 3) break;
                }
            }
            cnt++;
        }

        out_segments[seg_count++] = {
            SEG_STRAIGHT,
            d,
            DIR_INVALID,
            d,
            cnt,
            path[i].x, path[i].y,
            path[i + cnt].x, path[i + cnt].y
        };
        i += cnt;
    }

    return seg_count;
}
