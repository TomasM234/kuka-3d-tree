import yourdfpy
from ikpy.chain import Chain

urdf_path = r"d:\3DTisk\KUKA\kuka_kr16_support\urdf\kr16_2.urdf"

try:
    print("Testing yourdfpy...")
    urdf = yourdfpy.URDF.load(urdf_path)
    print("Robot name:", urdf.robot.name)
    print("Links:", [l.name for l in urdf.robot.links])
    print("Joints:", [j.name for j in urdf.robot.joints])
    for link in urdf.robot.links:
        for visual in link.visuals:
            if visual.geometry.mesh:
                print(f"Visual {link.name}: {visual.geometry.mesh.filename}")
                
    print("\nTesting ikpy...")
    # ikpy expects active links to be appropriately defined
    chain = Chain.from_urdf_file(urdf_path)
    print("IK Links:", [l.name for l in chain.links])
    print("Active links mask:", chain.active_links_mask)
    sol = chain.inverse_kinematics([0.8, 0.0, 0.5])
    print("Solution:", sol)
except Exception as e:
    import traceback
    traceback.print_exc()
