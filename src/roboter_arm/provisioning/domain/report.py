"""When a read-only Pi inspection counts as passed."""


def report_valid(report, *, app):
    valid = report['files_match'] and report['runtime']['valid'] and report['machine'] == 'aarch64' and report['i2c_access']
    if app:
        valid = valid and report['app']['reachable'] and report['app']['assets_match']
    return valid
