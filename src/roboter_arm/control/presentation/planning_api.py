"""Trajectory planning routes of the mock controller; they plan and rehearse, and never move the arm."""
from fastapi.responses import JSONResponse

from roboter_arm.control.presentation import api_models as body

REFUSALS = {status: {'model': body.Refused} for status in (400, 500)}


def register(app, planner):
    @app.get('/api/sim/planning', tags=['Simulation'], responses=REFUSALS,
             summary='Trajectory planning defaults from planning.json: parameters, scene and their rules')
    def planning():
        try:
            return planner.defaults()
        except ValueError as error:
            return JSONResponse({'error': str(error)}, 500)

    @app.post('/api/sim/plan', tags=['Simulation'], responses=REFUSALS,
              summary='Plan a task and rehearse it in a copy of the cube scene; sends no servo command')
    def plan(args: body.Plan):
        try:
            return planner.plan(**args.model_dump(exclude_unset=True))
        except ValueError as error:
            return JSONResponse({'error': str(error)}, 400)
