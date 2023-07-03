#!/bin/bash

cd wheelhouse
first_file=(*)

echo "Using $first_file to determiate package version"
version_to_delete=$(echo $first_file | cut -d - -f 2)
python=$(cat <<-END
import sys
import json

version = sys.argv[1]

for package in json.load(sys.stdin):
    if package["version"] == version:
        print(package["id"])
        exit(0)

exit(1)
END
)
api="${CI_API_V4_URL}"
project_id="${CI_PROJECT_ID}"
package_name="luhsoccer-baguette"
header="JOB-TOKEN: ${CI_JOB_TOKEN}"

id=$(curl --header "$header" "$api/projects/$project_id/packages?package_type=pypi&package_name=$package_name" | \
    python3 -c "$python" $version_to_delete)

if [ $? -eq 0 ]; then
    echo "Found old package with same version. Deleting..."
    curl --request DELETE --header "$header" "$api/projects/$project_id/packages/$id"
else 
    echo "Package does not exists. We have a new version!!"
fi

cd ..
TWINE_PASSWORD=${CI_JOB_TOKEN} TWINE_USERNAME=gitlab-ci-token python3 -m twine upload --repository-url ${CI_API_V4_URL}/projects/${CI_PROJECT_ID}/packages/pypi wheelhouse/*