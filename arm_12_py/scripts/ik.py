import math

def inverse_kinematics(self, T03: float, T13: float, T23: float, theta4: float):
    for computed_a in range(1, 801, 1):

        try:
            # print("03: {0}, 13: {1}, 23: {2}, theta4: {3}".format(T03, T13, T23, theta4))
            B = T23 - self.d4 * math.sin(theta4 * math.pi / 180) + (T13 - self.Py) * self.t10 - self.h - self.e1; # sin(theta4*PI/180)
            # print(f'B: {B}')
            #computed_b = 2 * math.sqrt(abs(self.d1 * self.d1 - B * B)) - self.e3 # 21.3 
            computed_b = 2 * math.sqrt(abs(self.d1 * self.d1 - B * B)) + self.e2 * 2
            #Px = computed_a + 30 + (computed_b + self.e3) / 2 + self.e2
            sol = computed_b + computed_a - self.e4 - self.e3
            Px = computed_a + self.e4 / 2 + computed_b / 2 
            y1 = (T13 - self.Py) / self.c10
            x1 = T03 - Px
            d44 = self.d4 * math.cos(theta4 * math.pi / 180)
            csphy = ((self.d3 + d44) * (self.d3 + d44) - self.d2 * self.d2 - (x1 * x1 + y1 * y1)) / (-2 * self.d2 * math.sqrt(x1 * x1 + y1 * y1))
        # if
            if computed_a > self.computed_a_mix and computed_b > self.computed_mix \
                and computed_b < self.computed_max \
                    and ( computed_a + self.e4 / 2 + computed_b / 2 ) <= self.num1 \
                        and csphy >= -1.0 and csphy <= 1.0 and sol < self.num2:
                solution = {}
                phy = math.acos(csphy) * 180 / math.pi
                c3 = (x1 * x1 + y1 * y1 - (self.d3 + d44) * (self.d3 + d44) - self.d2 * self.d2) / (2 * self.d2 * (self.d3 + d44))
                solution['a'] = abs(computed_a)
                #solution['b'] = computed_b + computed_a - (self.e3 + 60) + 60 
                solution['b'] = sol
                solution['B'] = B
                if T03 < Px:
                    solution["theta2"] = math.atan2(y1, x1) * 180 / math.pi + phy
                    solution["theta3"] = -math.acos(c3) * 180 / math.pi
                else:
                    solution["theta2"] = math.atan2(y1, x1) * 180 / math.pi - phy
                    solution["theta3"] = math.acos(c3) * 180 / math.pi
                self.results.append(solution)
        except Exception as e:
            print(e)
    return self.results


