"""Unit tests for choosing new fast-set files from Thingi10K's metadata (no downloads)."""

import unittest

from benchmarks.add_fast_files import draw_order, load_candidates, problems_of
from benchmarks.runner import MeshMetrics

SUMMARY = """ID,Thing ID,License,Link,No duplicated faces,Closed,Edge manifold,No degenerate faces,Vertex manifold,Single Component,PWN
1,10,CC-BY,https://x/a.stl,TRUE,TRUE,TRUE,TRUE,TRUE,TRUE,TRUE
2,20,CC-BY,https://x/b.stl,TRUE,FALSE,TRUE,TRUE,TRUE,TRUE,FALSE
3,30,CC-BY,https://x/c.obj,TRUE,TRUE,TRUE,TRUE,TRUE,TRUE,TRUE
4,40,CC-BY,https://x/d.stl,TRUE,TRUE,TRUE,TRUE,TRUE,TRUE,TRUE
5,50,CC-BY,https://x/e.stl,TRUE,TRUE,TRUE,TRUE,TRUE,TRUE,TRUE
6,60,CC-BY,https://x/f.stl,TRUE,TRUE,TRUE,TRUE,TRUE,TRUE,TRUE
"""
GEOMETRY = """file_id,num_faces,num_self_intersections
1,100,0
2,200,0
3,300,0
4,400,5
5,999999,0
6,600,0
"""
CONTEXT = """Thing ID,Date,Category,Sub-category,Name,Author,License
10,2011,None,None,A,alice,CC-BY
"""


class TestAddFastFiles(unittest.TestCase):
    def test_candidates_filter_and_label(self):
        c = load_candidates(SUMMARY, GEOMETRY, CONTEXT, max_faces=1000, exclude={"6"})
        # 3 is not an STL, 5 is over the face limit, 6 is already listed
        self.assertEqual([x.file_id for x in c], [1, 2, 4])
        self.assertEqual([x.healthy for x in c], [True, False, False])  # 2 is open, 4 crosses itself
        self.assertEqual(c[0].author, "alice")
        self.assertEqual(c[0].thing_id, "10")

    def test_draw_is_repeatable_and_mostly_broken(self):
        from benchmarks.add_fast_files import Candidate

        pool = [Candidate(i, str(i), 10, i % 2 == 0, "", "") for i in range(100)]
        a = draw_order(pool, seed=7, healthy_share=0.2)
        b = draw_order(pool, seed=7, healthy_share=0.2)
        self.assertEqual([x.file_id for x in a], [x.file_id for x in b])
        self.assertNotEqual([x.file_id for x in a], [x.file_id for x in draw_order(pool, seed=8, healthy_share=0.2)])
        self.assertEqual(sum(x.healthy for x in a[:10]), 2)
        self.assertEqual(len(a), 100)

    def test_problem_labels(self):
        self.assertEqual(problems_of(MeshMetrics(watertight_before=True)), [])
        m = MeshMetrics(watertight_before=False, open_edges_before=3, self_intersecting_faces_before=2, duplicate_faces_before=1)
        self.assertEqual(problems_of(m), ["holes", "self-intersections", "duplicate-or-degenerate"])


if __name__ == "__main__":
    unittest.main()
