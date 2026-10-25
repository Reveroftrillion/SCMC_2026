"""A late road-rejected return must not hide an earlier road-valid return."""
from test_tunnel_simple import Box, make_plan


def test_earlier_return_is_tried_when_preferred_late_return_leaves_pavement():
    checks = []

    def segment_ok(d0, d1, start, length, s_from, s_to):
        checks.append((d0, d1, start, s_to))
        # The bypass lane ends at s70. The requested late return starts at s80.
        return d1 != 0. or s_to <= 70.

    result = make_plan([[Box(40., 42., -.5, .5)]], lambda _: [2.],
                       return_from=lambda d: 80., segment_ok=segment_ok)
    assert any(d1 == 0. and start >= 80. for _, d1, start, _ in checks)
    assert result.blocked_at is None
    assert result.offsets[-1] == 0.
    assert result.starts[-1] + result.lengths[-1] + 3. <= 70.


def test_every_return_outside_pavement_still_blocks_the_chain():
    result = make_plan([[Box(40., 42., -.5, .5)]], lambda _: [2.],
                       return_from=lambda d: 80., segment_ok=lambda d0, d1, *args: d1 != 0.)
    assert result.blocked_at == 0


def test_invalid_entry_cannot_be_rescued_by_a_valid_return():
    result = make_plan([[Box(40., 42., -.5, .5)]], lambda _: [2.],
                       return_from=lambda d: 80., segment_ok=lambda d0, d1, *args: d1 == 0.)
    assert result.blocked_at == 0


def test_valid_late_return_keeps_its_original_preference():
    result = make_plan([[Box(40., 42., -.5, .5)]], lambda _: [2.],
                       return_from=lambda d: 80., segment_ok=lambda *args: True)
    assert result.blocked_at is None
    assert result.starts[-1] >= 80.
