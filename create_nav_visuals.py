import os
import glob
from PIL import Image, ImageDraw, ImageFont
import shutil

frames_dir = '/mnt/c/Users/Jaya/Downloads/RC/nav_frames'
artifact_dir = '/mnt/c/Users/Jaya/.gemini/antigravity/brain/dc89a3f2-e3df-4a74-952a-9d29442fd5b3'
all_frames = sorted(glob.glob(os.path.join(frames_dir, 'frame_*.png')))

print(f"Total frames available: {len(all_frames)}")

if all_frames:
    # 1. Select key milestone frames for a 6-panel montage
    # Milestones across the 100+ frames:
    indices = [
        0,                      # Start
        len(all_frames) // 5,   # Turning
        2 * len(all_frames) // 5, # Corridor drive
        3 * len(all_frames) // 5, # Room entrance
        4 * len(all_frames) // 5, # Navigating loop
        len(all_frames) - 1     # Goal arrival
    ]
    
    milestone_images = [Image.open(all_frames[i]) for i in indices]
    labels = [
        "Stage 1: Launch & Hallway Start",
        "Stage 2: Ackermann Cornering (RPP)",
        "Stage 3: Hallway Traversal",
        "Stage 4: Room Entry Navigation",
        "Stage 5: Autonomous Loop Re-routing",
        "Stage 6: Goal Reached & Braking"
    ]

    # Create 3x2 grid montage
    panel_w, panel_h = 640, 360
    montage = Image.new('RGB', (panel_w * 3, panel_h * 2 + 80), color=(20, 20, 25))
    draw = ImageDraw.Draw(montage)
    
    # Title banner
    title = "Autonomous Ackermann Navigation Validation (Smac Hybrid-A* + Pure Pursuit Controller)"
    draw.text((30, 25), title, fill=(0, 255, 180))

    for idx, (img, label) in enumerate(zip(milestone_images, labels)):
        col = idx % 3
        row = idx // 3
        x_pos = col * panel_w
        y_pos = 70 + row * panel_h
        
        resized = img.resize((panel_w - 10, panel_h - 40))
        montage.paste(resized, (x_pos + 5, y_pos + 5))
        
        # Draw label bar
        draw.rectangle([(x_pos + 5, y_pos + panel_h - 32), (x_pos + panel_w - 5, y_pos + panel_h - 5)], fill=(30, 30, 40))
        draw.text((x_pos + 15, y_pos + panel_h - 26), label, fill=(255, 255, 255))

    montage_path = '/mnt/c/Users/Jaya/Downloads/RC/robot_movement_montage.png'
    montage.save(montage_path)
    print(f"Saved montage to {montage_path}")
    
    # Copy to artifacts
    shutil.copy(montage_path, os.path.join(artifact_dir, 'robot_movement_montage.png'))
    shutil.copy(all_frames[len(all_frames)//2], os.path.join(artifact_dir, 'robot_moving_action.png'))
    shutil.copy(all_frames[-1], os.path.join(artifact_dir, 'robot_goal_reached.png'))
    print("Copied images to artifact directory.")

    # 2. Build animated GIF (sample every 2nd frame)
    gif_frames = []
    for f in all_frames[::2]:
        im = Image.open(f).resize((480, 270))
        gif_frames.append(im)

    gif_path = '/mnt/c/Users/Jaya/Downloads/RC/robot_autonomous_drive.gif'
    gif_frames[0].save(
        gif_path,
        save_all=True,
        append_images=gif_frames[1:],
        duration=150,
        loop=0
    )
    shutil.copy(gif_path, os.path.join(artifact_dir, 'robot_autonomous_drive.gif'))
    print(f"Saved and copied animated GIF ({len(gif_frames)} frames) to {gif_path}")
