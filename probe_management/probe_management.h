// Probe Interface

enum ProbeAction {
  ProbeStay          = 0,
  ProbeDeploy        = BIT(0),
  ProbeStow          = BIT(1),
  ProbeDeployAndStow = (ProbeDeploy | ProbeStow)
};

extern bool z_probe_is_active;

extern void stow_z_probe(bool doRaise = true);
extern void deploy_z_probe();

// Probe bed height at position (x,y), returns the measured z value
extern float probe_pt(float x, float y, float z_before, ProbeAction probe_action = ProbeDeployAndStow, int verbose_level = 1);
