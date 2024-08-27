#!/usr/bin/env python3

import os
from util import get_project, verify_env_var_presence
from datetime import date, datetime, timedelta
from gitlab.v4.objects.packages import ProjectPackage

if __name__ == "__main__":
    verify_env_var_presence("DELETE_OLDER_THAN_X_DAYS")
    days_to_delete = os.environ["DELETE_OLDER_THAN_X_DAYS"]
    project = get_project()

    packages: list[ProjectPackage] = project.packages.list(
        package_type="pypi", order_by="created_at", sort="asc", get_all=True
    )

    oldest_date = datetime.now() - timedelta(days=int(days_to_delete))
    print(f"Remove packages older than {oldest_date}")
    for package in packages:
        date_of_creation = datetime.strptime(
            package.created_at, "%Y-%m-%dT%H:%M:%S.%fZ"
        )
        old = oldest_date > date_of_creation

        if old and "dev" in package.version:
            # delete package
            print(f"Deleting {package.name}-{package.version}...")
            package.delete()
