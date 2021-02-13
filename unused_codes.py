    """
    This function returns distance to the lead vehicle
    """

    def get_road_dist(self,collission_idx,path):
        mask = np.ones(collission_idx-1,path.shape[0],1)-np.ones(collission_idx-1,path.shape[0])
        diff_points = mask@path

        diff_points = np.square(diff_points)

        distance_lead = np.sum(np.sum(diff_points,axis =1))
        
        return distance_lead