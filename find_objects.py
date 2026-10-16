import re

model_file = "/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_house/model.sdf"
with open(model_file, "r") as f:
    sdf_content = f.read()

# Find each model and its pose inside model.sdf
# In SDF, a nested model usually has:
# <model name='first_2015_trash_can'>
#   <pose frame=''>X Y Z R P Y</pose>
# or similar.
pattern = r"<model name=[\'\"](first_2015_trash_can.*?)[\'\"]>([\s\S]*?)</model>"
matches = re.findall(pattern, sdf_content)

for name, body in matches:
    print(f"=== {name} ===")
    poses = re.findall(r"<pose.*?>(.*?)</pose>", body)
    for p in poses:
        print("  Pose:", p.strip())

# Also let's find all models and their top-level pose tag
all_model_blocks = re.findall(r"(<model name=[\'\"][^\'\"]+[\'\"]>[\s\S]*?)(?=<model name=|\Z)", sdf_content)
print(f"\nTotal model blocks: {len(all_model_blocks)}")
for b in all_model_blocks:
    m_name = re.search(r"<model name=[\'\"]([^\'\"]+)[\'\"]>", b)
    m_pose = re.search(r"<pose[^>]*>([^<]+)</pose>", b)
    if m_name:
        name = m_name.group(1)
        pose = m_pose.group(1).strip() if m_pose else "None"
        print(f"Model: {name:25s} | Pose: {pose}")
