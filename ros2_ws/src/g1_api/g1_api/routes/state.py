from __future__ import annotations

from fastapi import APIRouter, Request


router = APIRouter()


@router.get("/api/skills")
def get_skills(request: Request) -> dict[str, object]:
    return {"skills": request.app.state.api_node.get_skills()}


@router.get("/api/state")
def get_state(request: Request) -> dict[str, object]:
    return request.app.state.api_node.get_state_snapshot()
