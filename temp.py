    loc = carla.Location(x = -195, y=90,z = 0 )
    world.world.debug.draw_string(loc, 'X', draw_shadow=False,color=carla.Color(r=255, g=0, b=0), life_time=100,persistent_lines=True)

    waypoint = world_map.get_waypoint(loc,project_to_road = True,lane_type = (carla.LaneType.Driving|carla.LaneType.Sidewalk|carla.LaneType.Parking|carla.LaneType.Parking))
    
    junc_points = print_junction(world,waypoint)
    
    lines = get_line(junc_points)
    # draw_hex(world,lines)
    inter_junc_points = solve_lines(lines)

    box_points = get_box(world_map,inter_junc_points)
    
    for i in range(box_points.shape[0]):
        
        world.world.debug.draw_string(carla.Location(x=box_points[i,0],y = box_points[i,1],z = 1),"A", draw_shadow=False,color=carla.Color(r=255, g=255, b=0), life_time=10000,persistent_lines=True)
    
    

    # print()
    loc = carla.Location(x = -120, y=0,z = 0 )
    world.world.debug.draw_string(loc, 'X', draw_shadow=False,color=carla.Color(r=255, g=0, b=0), life_time=100,persistent_lines=True)

    waypoint = world_map.get_waypoint(loc,project_to_road = True,lane_type = (carla.LaneType.Driving|carla.LaneType.Sidewalk|carla.LaneType.Parking|carla.LaneType.Parking))
    
    junc_points = print_junction(world,waypoint)
    # print(junc_points.shape)
    lines = get_line(junc_points)

    # print(lines)
    # draw_hex(world,lines)
    inter_junc_points = solve_lines(lines)

    box_points = get_box(world_map,inter_junc_points)
    
    for i in range(box_points.shape[0]):

        world.world.debug.draw_string(carla.Location(x=box_points[i,0],y = box_points[i,1],z = 1),"A", draw_shadow=False,color=carla.Color(r=255, g=255, b=0), life_time=10000,persistent_lines=True)
    # print()