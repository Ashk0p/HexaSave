import math

class HexapodIK:
    def __init__(self, coxa_len=0.05, femur_len=0.10, tibia_len=0.15):
        self.l1 = coxa_len
        self.l2 = femur_len
        self.l3 = tibia_len

    def solve(self, x, y, z):
        """
        Calculates joint angles for a leg given target tip coordinates (x, y, z).
        z is negative when foot is below hip center.
        """
        # 1. Coxa (yaw) angle
        coxa = math.atan2(y, x)

        # 2. Planar projection distance from coxa joint to tip
        r_total = math.hypot(x, y)
        r = r_total - self.l1

        # 3. Direct distance from femur joint to foot tip
        d = math.hypot(r, z)

        # Workspace clamping to avoid math domain errors
        d_max = self.l2 + self.l3 - 0.001
        d_min = abs(self.l2 - self.l3) + 0.001
        d = max(d_min, min(d_max, d))

        # 4. Trigonometric internal angles (Law of Cosines)
        cos_femur2 = (self.l2**2 + d**2 - self.l3**2) / (2 * self.l2 * d)
        cos_tibia = (self.l2**2 + self.l3**2 - d**2) / (2 * self.l2 * self.l3)

        cos_femur2 = max(-1.0, min(1.0, cos_femur2))
        cos_tibia = max(-1.0, min(1.0, cos_tibia))

        beta = math.acos(cos_femur2)
        alpha = math.atan2(z, r)

        # Joint output values (in radians)
        femur = alpha + beta
        tibia = -(math.pi - math.acos(cos_tibia))

        return coxa, femur, tibia