"""Tests for the _mesheal Python module. Synthetic geometry only; numpy required."""
import time
import unittest

import numpy as np

import _mesheal as m


def cube():
    v = np.array([[x, y, z] for z in (0, 1) for y in (0, 1) for x in (0, 1)], dtype=np.float64)
    # index = x + 2y + 4z; outward-facing CCW triangles
    f = np.array([
        [0, 2, 3], [0, 3, 1],  # z = 0 (bottom)
        [4, 5, 7], [4, 7, 6],  # z = 1 (top)
        [0, 1, 5], [0, 5, 4],  # y = 0
        [2, 6, 7], [2, 7, 3],  # y = 1
        [0, 4, 6], [0, 6, 2],  # x = 0
        [1, 3, 7], [1, 7, 5],  # x = 1
    ], dtype=np.int32)
    return v, f


def sphere(n_lat=16, n_lon=24, radius=1.0):
    verts = [[0.0, 0.0, radius]]
    for i in range(1, n_lat):
        th = np.pi * i / n_lat
        for j in range(n_lon):
            ph = 2 * np.pi * j / n_lon
            verts.append([radius * np.sin(th) * np.cos(ph), radius * np.sin(th) * np.sin(ph), radius * np.cos(th)])
    verts.append([0.0, 0.0, -radius])
    faces = []
    south = len(verts) - 1
    for j in range(n_lon):
        faces.append([0, 1 + j, 1 + (j + 1) % n_lon])
    for i in range(n_lat - 2):
        a = 1 + i * n_lon
        b = a + n_lon
        for j in range(n_lon):
            j2 = (j + 1) % n_lon
            faces.append([a + j, b + j, b + j2])
            faces.append([a + j, b + j2, a + j2])
    last = 1 + (n_lat - 2) * n_lon
    for j in range(n_lon):
        faces.append([south, last + (j + 1) % n_lon, last + j])
    return np.array(verts, dtype=np.float64), np.array(faces, dtype=np.int32)


def by_cat(results):
    return {r.category: r for r in results}


def open_cube():
    v, f = cube()
    return v, f[:-1].copy()


class CheckTests(unittest.TestCase):
    def test_closed_cube_is_clean(self):
        v, f = cube()
        res = m.check(v, f)
        self.assertEqual(len(res), 10)
        for r in res:
            if r.category == m.Category.SHELLS:
                self.assertEqual(r.info_count, 1)
            else:
                self.assertEqual(r.count, 0, r.name)

    def test_open_cube_has_one_hole(self):
        v, f = open_cube()
        r = by_cat(m.check(v, f))[m.Category.OPEN_EDGES]
        self.assertEqual(r.count, 1)
        self.assertGreater(len(r.faces), 0)
        self.assertEqual(r.faces.dtype, np.int64)

    def test_duplicate_triangle(self):
        v, f = cube()
        f = np.vstack([f, f[:1]])
        self.assertEqual(by_cat(m.check(v, f))[m.Category.DUPLICATES].count, 1)

    def test_flipped_triangle(self):
        v, f = cube()
        f = f.copy()
        f[3] = f[3][::-1]
        self.assertGreater(by_cat(m.check(v, f))[m.Category.FLIPPED_NORMALS].count, 0)

    def test_category_subset(self):
        v, f = cube()
        res = m.check(v, f, categories=[m.Category.SHELLS])
        self.assertEqual(len(res), 1)


class RepairTests(unittest.TestCase):
    def test_fix_all_closes_open_cube(self):
        v, f = open_cube()
        r = m.repair(v, f, m.Method.FIX_ALL)
        self.assertTrue(r.success, r.status)
        self.assertGreater(len(r.report), 0)
        again = by_cat(m.check(r.vertices, r.faces))
        self.assertEqual(again[m.Category.OPEN_EDGES].count, 0)

    def test_fix_all_with_mask(self):
        v, f = open_cube()
        mask = np.ones(len(f), dtype=np.uint8)
        r = m.repair(v, f, m.Method.FIX_ALL, face_in_scope=mask)
        self.assertTrue(r.success, r.status)

    def test_decimate_half(self):
        v, f = sphere(24, 36)
        p = m.RepairParams()
        p.decimate_ratio = 0.5
        r = m.repair(v, f, m.Method.DECIMATE, p)
        self.assertTrue(r.success, r.status)
        ratio = len(r.faces) / len(f)
        self.assertTrue(0.35 < ratio < 0.65, ratio)

    def test_decimate_with_mask_keeps_outside(self):
        v, f = sphere(24, 36)
        mask = np.zeros(len(f), dtype=np.uint8)
        mask[: len(f) // 2] = 1
        r = m.repair(v, f, m.Method.DECIMATE, face_in_scope=mask)
        self.assertTrue(r.success, r.status)
        out = {tuple(sorted(map(tuple, r.vertices[tri].round(9).tolist()))) for tri in r.faces}
        for i in np.nonzero(mask == 0)[0]:
            key = tuple(sorted(map(tuple, v[f[i]].round(9).tolist())))
            self.assertIn(key, out)

    def test_merge_soup_cube(self):
        v, f = cube()
        soup_v = v[f].reshape(-1, 3).copy()
        soup_f = np.arange(len(soup_v), dtype=np.int32).reshape(-1, 3)
        self.assertEqual(len(soup_v), 36)
        p = m.RepairParams()
        p.merge_preserve_open_edges = False  # every edge of a soup is open
        p.merge_keep_surfaces_apart = False
        r = m.repair(soup_v, soup_f, m.Method.MERGE, p)
        self.assertTrue(r.success, r.status)
        self.assertEqual(len(r.vertices), 8)

    def test_voxel_and_alpha_wrap(self):
        v, f = sphere(16, 24)
        for method in (m.Method.VOXEL, m.Method.ALPHA_WRAP):
            r = m.repair(v, f, method)
            self.assertTrue(r.success, (method, r.status))
            self.assertGreater(len(r.faces), 0)
            c = by_cat(m.check(r.vertices, r.faces))
            self.assertEqual(c[m.Category.OPEN_EDGES].count, 0, method)
            self.assertEqual(c[m.Category.NON_MANIFOLD_EDGES].count, 0, method)


class DefaultsAndValidation(unittest.TestCase):
    def test_defaults(self):
        self.assertEqual(m.RepairParams().decimate_ratio, 0.5)
        self.assertEqual(m.FixAllParams().passes, 3)
        self.assertEqual(m.CheckParams().thin_max_aspect, 100.0)
        p = m.RepairParams()
        p.check.thin_max_aspect = 50.0
        p.fix_all.passes = 2
        self.assertEqual(p.check.thin_max_aspect, 50.0)
        self.assertEqual(p.fix_all.passes, 2)

    def test_version(self):
        self.assertIsInstance(m.version(), str)

    def test_bad_shapes(self):
        v, f = cube()
        with self.assertRaises((TypeError, ValueError)):
            m.check(v[:, :2].copy(), f)
        with self.assertRaises((TypeError, ValueError)):
            m.check(v, f[:, :2].copy())

    def test_bad_index(self):
        v, f = cube()
        f = f.copy()
        f[0, 0] = 99
        with self.assertRaises(ValueError):
            m.check(v, f)
        f[0, 0] = -1
        with self.assertRaises(ValueError):
            m.check(v, f)

    def test_bad_mask_length(self):
        v, f = cube()
        with self.assertRaises(ValueError):
            m.repair(v, f, m.Method.DECIMATE, face_in_scope=np.ones(3, dtype=np.uint8))


class JobTests(unittest.TestCase):
    def test_repair_job_matches_sync(self):
        v, f = open_cube()
        job = m.start_repair(v, f, m.Method.FIX_ALL)
        self.assertTrue(job.wait(60))
        self.assertTrue(job.done)
        self.assertTrue(0.0 <= job.progress <= 1.0)
        a = job.result()
        b = m.repair(v, f, m.Method.FIX_ALL)
        self.assertEqual(a.success, b.success)
        self.assertEqual(a.faces.shape, b.faces.shape)
        self.assertEqual(a.vertices.shape, b.vertices.shape)

    def test_check_job(self):
        v, f = open_cube()
        job = m.start_check(v, f)
        seen = []
        while not job.done:
            seen.append(job.progress)
            time.sleep(0.001)
        self.assertTrue(job.wait(60))
        self.assertTrue(all(0.0 <= p <= 1.0 for p in seen))
        res = by_cat(job.result())
        self.assertEqual(res[m.Category.OPEN_EDGES].count, 1)

    def test_result_before_done_raises(self):
        v, f = sphere(48, 72)
        p = m.RepairParams()
        p.remesh_target_size = 0.01
        job = m.start_repair(v, f, m.Method.VOXEL, p)
        try:
            with self.assertRaises(RuntimeError):
                job.result()
        finally:
            job.cancel()
            job.wait(60)

    def test_cancel_voxel(self):
        v, f = sphere(48, 72)
        p = m.RepairParams()
        p.remesh_target_size = 0.01
        job = m.start_repair(v, f, m.Method.VOXEL, p)
        t0 = time.time()
        job.cancel()
        self.assertTrue(job.wait(60))
        self.assertLess(time.time() - t0, 30)
        r = job.result()
        self.assertTrue(r.cancelled or r.success)


if __name__ == "__main__":
    unittest.main()
