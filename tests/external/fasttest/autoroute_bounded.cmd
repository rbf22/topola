{
  "done": [
    {
      "MultilayerAutoroute": [
        {"pins": [], "net_names": [], "nets_excluded": true},
        {
          "anterouter": {"fanout_clearance": 2.5},
          "planar": {
            "principal_layer": 0,
            "presort_by": "RatlineIntersectionCountAndLength",
            "permutate": true,
            "router": {
              "routed_band_width": 1.7767,
              "wrap_around_bands": true,
              "squeeze_through_under_bends": true
            },
            "timeout": {"initial": 0.5, "progress_bonus": 0.001},
            "max_reconfigurations": 5
          },
          "timeout": {"initial": 5.0, "progress_bonus": 0.05},
          "layer_seed": 0
        }
      ]
    }
  ],
  "undone": []
}
