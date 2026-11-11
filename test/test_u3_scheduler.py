"""U3 PatrolScheduler unit tests (UT-U3-01 ~ 05)."""
from collections import Counter

from ugv_patrol_planner.mission.scheduler import PatrolScheduler


def make(n=3) -> PatrolScheduler:
    s = PatrolScheduler()
    for i in range(n):
        s.add_point((i, i))
    return s


def test_ut_u3_01_add_and_remove():
    s = make(3)
    assert s.remove_point('P2')
    assert not s.remove_point('P9')
    assert [p.pid for p in s.snapshot()] == ['P1', 'P3']
    assert s.add_point((0, 0)) == 'P1'  # same cell keeps its ID
    assert len(s) == 2


def test_ut_u3_02_longest_wait_first():
    s = make(3)
    for pid, t in (('P1', 10), ('P2', 5), ('P3', 8)):
        s.mark_visited(pid, t)
    assert s.select_next(20).pid == 'P2'
    assert s.select_next(20, exclude={'P2'}).pid == 'P3'


def test_ut_u3_03_unvisited_first():
    s = make(3)
    s.mark_visited('P1', 1)
    assert s.select_next(100).pid == 'P2'


def test_ut_u3_04_mark_visited():
    s = make(3)
    for pid, t in (('P1', 10), ('P2', 5), ('P3', 8)):
        s.mark_visited(pid, t)
    s.mark_visited('P2', 30)
    assert s.get('P2').last_visit == 30
    assert s.select_next(31).pid != 'P2'


def test_ut_u3_05_even_coverage():
    s = make(4)
    visits = Counter()
    for t in range(40):
        p = s.select_next(t)
        visits[p.pid] += 1
        s.mark_visited(p.pid, t)
    assert max(visits.values()) - min(visits.values()) <= 1
