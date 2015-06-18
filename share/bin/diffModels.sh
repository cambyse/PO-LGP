#!/bin/bash

# abort on erros
set -e

echo "======================================================================"
echo "Diffing your local model files with the PR2."
echo "Each diff should be empy!"
echo "I assume you can connect to the PR2 with the alias 'ssh pr2'"
echo "======================================================================"
echo "diff ~/.ros/model.kvg <(ssh pr2 'cat ~/.ros/model.kvg')"
diff ~/.ros/model.kvg <(ssh pr2 'cat ~/.ros/model.kvg')

echo "======================================================================"
echo "diff ~/.ros/pr2_model/pr2_clean.ors <(ssh pr2 'cat ~/.ros/pr2_model/pr2_clean.ors')"
diff ~/.ros/pr2_model/pr2_clean.ors <(ssh pr2 'cat ~/.ros/pr2_model/pr2_clean.ors')

echo "======================================================================"
echo "diff ~/.ros/pr2_model/pr2_modifications.ors <(ssh pr2 'cat ~/.ros/pr2_model/pr2_modifications.ors')"
diff ~/.ros/pr2_model/pr2_modifications.ors <(ssh pr2 'cat ~/.ros/pr2_model/pr2_modifications.ors')
