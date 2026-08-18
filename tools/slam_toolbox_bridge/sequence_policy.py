"""Pure sequence/session policy shared by the bridge and its host tests."""

from __future__ import annotations


def classify_packet(
    active_session: str,
    last_sequence: int,
    incoming_session: str,
    incoming_sequence: int,
) -> str:
    if incoming_session != active_session:
        return "new_session"
    if incoming_sequence <= last_sequence:
        return "stale"
    return "accept"
