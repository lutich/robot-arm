"""Fresh distance batches through the existing API, independently of servo actions."""
from fastapi import Query, Request
from fastapi.responses import JSONResponse

from roboter_arm.control.domain.distance import DistanceConflict, DistanceUnavailable
from roboter_arm.control.presentation import api_models as body

DISTANCE_PATHS = ('/api/distance/status', '/api/distance/read')
DISTANCE_FAILURE = 'Distance request failed'
REFUSALS = {status: {'model': body.Refused} for status in (400, 409, 503, 500)}


def refusal(error):
    if isinstance(error, DistanceConflict):
        status = 409
    elif isinstance(error, DistanceUnavailable):
        status = 503
    else:
        return JSONResponse({'error': DISTANCE_FAILURE}, 500)
    return JSONResponse({'error': str(error)}, status)


def register(app, distance):
    @app.get('/api/distance/status', tags=['Distance'], response_model=body.DistanceStatus,
             responses=REFUSALS, summary='Sensor readiness and fixed defaults; does not trigger a reading')
    def status():
        try:
            return body.DistanceStatus.model_validate(distance.status())
        except Exception as error:
            return refusal(error)

    @app.get('/api/distance/read', tags=['Distance'], response_model=body.DistanceBatch,
             responses=REFUSALS, summary='Acquire a fresh batch; return median, spread and individual readings')
    def read(request: Request, samples: int = Query(default=5, ge=1, le=20,
                                                   description='Number of attempts, from 1 to 20')):
        if set(request.query_params) - {'samples'}:
            return JSONResponse({'error': 'Only the samples query parameter is supported'}, 400)
        values = request.query_params.getlist('samples')
        if values and (len(values) != 1 or not values[0].isascii() or not values[0].isdecimal()):
            return JSONResponse({'error': 'samples must be an integer from 1 to 20'}, 400)
        try:
            return body.DistanceBatch.model_validate(distance.read(samples=samples))
        except Exception as error:
            return refusal(error)
