from __future__ import annotations

from pydantic import BaseModel
from fastapi import APIRouter, Request


router = APIRouter()


class CommandRequest(BaseModel):
    command: str


@router.post("/api/command")
def post_command(payload: CommandRequest, request: Request) -> dict[str, object]:
    return request.app.state.api_node.send_command(payload.command)
