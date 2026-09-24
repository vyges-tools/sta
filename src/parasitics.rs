// SPDX-License-Identifier: Apache-2.0
//! Parasitic reduction: an RC network to the driving-point pi model (`c2`–`rpi`–`c1`) and each
//! load's Elmore delay (O'Brien and Savarino, DAC 1989).

/// An RC network as built for one net: ground capacitance per node and resistors, each list in
/// creation order (the order the reduction walks them in).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Network {
    /// Ground capacitance per node, `f32` as stored.
    pub node_caps: Vec<f32>,
    /// `(n1, n2, ohms)`, `f32` as stored, in creation order.
    pub resistors: Vec<(usize, usize, f32)>,
}

/// The reduced model of one driver: the pi model and every load's Elmore delay, `(node, delay)`
/// in the order the second walk reaches the loads.
#[derive(Debug, Clone, PartialEq)]
pub struct PiElmore {
    pub c2: f32,
    pub rpi: f32,
    pub c1: f32,
    pub elmore: Vec<(usize, f32)>,
}

/// What a node is to the reduction: the pin capacitance it adds (a liberty port's for the
/// transition, a top-level port's external load; none without a pin — or when the network
/// already includes pin caps), and whether it is a LOAD pin (Elmore delays are kept for those).
pub struct NodePins<'a> {
    pub pin_cap: &'a dyn Fn(usize) -> f32,
    pub is_load: &'a dyn Fn(usize) -> bool,
}

/// The admittance moments by one depth-first walk from
/// the driver's node, the pi model from them, then the Elmore delays by a second walk.
///
/// Rules:
/// - a node's downstream capacitance is its ground cap plus its pin cap (and coupling caps — none
///   in an estimated network), summed in `double`, and STORED as `f32` for the second walk;
/// - each node's resistors are walked in creation order; one back to the node itself, or the one
///   arrived by, is skipped; one to a node already on the current path closes a LOOP and is
///   ignored in both walks;
/// - rules 3 and 4 in `double`: `y1 += yd1`, `y2 += yd2 − r·yd1²`, `y3 += yd3 − 2r·yd1·yd2 +
///   r²·yd1³`;
/// - with `y2 = y3 = 0` the load is purely capacitive (`c1 = y1`, `c2 = rpi = 0`); otherwise
///   `c1 = y2²/y3`, `c2 = y1 − y2²/y3`, `rpi = −y3²/y2³`, each narrowed to `f32`;
/// - a load's Elmore delay is the running `double` sum of `r · downstream cap` along the path,
///   each product in `f32` (both factors are), stored as `f32`.
pub fn reduce_to_pi_elmore(net: &Network, driver: usize, pins: &NodePins) -> PiElmore {
    let n = net.node_caps.len();
    let mut adjacency: Vec<Vec<usize>> = vec![Vec::new(); n];
    for (i, &(a, b, _)) in net.resistors.iter().enumerate() {
        adjacency[a].push(i);
        adjacency[b].push(i);
    }
    let mut w = Walk { net, adjacency: &adjacency, pins, visited: vec![false; n], loops: vec![false; net.resistors.len()], downstream: vec![0.0; n] };
    let (y1, y2, y3, _) = w.pi_dfs(driver, None);
    let (c2, rpi, c1) = if y2 == 0.0 && y3 == 0.0 {
        (0.0, 0.0, y1 as f32)
    } else {
        ((y1 - y2 * y2 / y3) as f32, (-y3 * y3 / (y2 * y2 * y2)) as f32, (y2 * y2 / y3) as f32)
    };
    let mut elmore = Vec::new();
    w.elmore_dfs(driver, None, 0.0, &mut elmore);
    PiElmore { c2, rpi, c1, elmore }
}

struct Walk<'a> {
    net: &'a Network,
    adjacency: &'a [Vec<usize>],
    pins: &'a NodePins<'a>,
    visited: Vec<bool>,
    loops: Vec<bool>,
    /// `node_values_`: each node's downstream capacitance, as `f32`.
    downstream: Vec<f32>,
}

impl Walk<'_> {
    fn other(&self, r: usize, node: usize) -> usize {
        let (a, b, _) = self.net.resistors[r];
        if node == a {
            b
        } else {
            a
        }
    }

    /// The admittance moments `(y1, y2, y3)` and downstream cap below `node`.
    fn pi_dfs(&mut self, node: usize, from: Option<usize>) -> (f64, f64, f64, f64) {
        let mut dwn_cap = f64::from(self.net.node_caps[node]) + f64::from((self.pins.pin_cap)(node));
        let (mut y1, mut y2, mut y3) = (dwn_cap, 0.0f64, 0.0f64);
        self.visited[node] = true;
        for &r in &self.adjacency[node] {
            if self.loops[r] {
                continue;
            }
            let other = self.other(r, node);
            if other == node || Some(r) == from {
                continue;
            }
            if self.visited[other] {
                self.loops[r] = true;
                continue;
            }
            let res = f64::from(self.net.resistors[r].2);
            let (yd1, yd2, yd3, dcap) = self.pi_dfs(other, Some(r));
            y1 += yd1;
            y2 += yd2 - res * yd1 * yd1;
            y3 += yd3 - 2.0 * res * yd1 * yd2 + res * res * yd1 * yd1 * yd1;
            dwn_cap += dcap;
        }
        self.downstream[node] = dwn_cap as f32;
        self.visited[node] = false;
        (y1, y2, y3, dwn_cap)
    }

    /// Each load's Elmore delay below `node`.
    fn elmore_dfs(&mut self, node: usize, from: Option<usize>, elmore: f64, out: &mut Vec<(usize, f32)>) {
        if from.is_some() && (self.pins.is_load)(node) {
            out.push((node, elmore as f32));
        }
        self.visited[node] = true;
        for &r in &self.adjacency[node] {
            let other = self.other(r, node);
            if Some(r) != from && !self.visited[other] && !self.loops[r] {
                let res = self.net.resistors[r].2;
                let node_elmore = elmore + f64::from(res * self.downstream[other]);
                self.elmore_dfs(other, Some(r), node_elmore, out);
            }
        }
        self.visited[node] = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // A driver (0) — 100 Ω — node 1 (1 fF) — 200 Ω — load 2 (2 fF): y1 = 3 fF; the moments by
    // rules 3 and 4 from the load up; the load's Elmore delay is 100·3f + 200·2f.
    #[test]
    fn a_chain_reduces_by_the_moment_rules() {
        let net = Network { node_caps: vec![0.0, 1e-15, 2e-15], resistors: vec![(0, 1, 100.0), (1, 2, 200.0)] };
        let pins = NodePins { pin_cap: &|_| 0.0, is_load: &|n| n == 2 };
        let p = reduce_to_pi_elmore(&net, 0, &pins);
        let (c1, c2) = (f64::from(1e-15f32), f64::from(2e-15f32));
        // The load: y = (2f, 0, 0); across 200 Ω: (2f, −200·4f², 200²·8f³); plus node 1's 1 fF.
        let (l1, l2, l3) = (c2, -200.0 * c2 * c2, 200.0 * 200.0 * c2 * c2 * c2);
        let (m1, m2, m3) = (c1 + l1, l2, l3);
        // Across 100 Ω to the driver.
        let (y1, y2, y3) = (m1, m2 - 100.0 * m1 * m1, m3 - 2.0 * 100.0 * m1 * m2 + 100.0 * 100.0 * m1 * m1 * m1);
        assert_eq!(p.c1, (y2 * y2 / y3) as f32);
        assert_eq!(p.c2, (y1 - y2 * y2 / y3) as f32);
        assert_eq!(p.rpi, (-y3 * y3 / (y2 * y2 * y2)) as f32);
        // Downstream caps are kept as f32; each r·cap product is f32.
        let e1 = f64::from(100.0f32 * ((c1 + c2) as f32));
        let e2 = e1 + f64::from(200.0f32 * 2e-15f32);
        assert_eq!(p.elmore, vec![(2, e2 as f32)]);
    }

    // A lumped load (no resistor): c1 is the capacitance, no pi.
    #[test]
    fn a_capacitive_load_is_all_c1() {
        let net = Network { node_caps: vec![5e-15], resistors: vec![] };
        let pins = NodePins { pin_cap: &|_| 1e-15, is_load: &|_| false };
        let p = reduce_to_pi_elmore(&net, 0, &pins);
        assert_eq!((p.c2, p.rpi, p.c1), (0.0, 0.0, (5e-15f32 as f64 + 1e-15f32 as f64) as f32));
    }

    // A resistor loop: the resistor that closes it is ignored in both walks; one back to its own
    // node is skipped.
    #[test]
    fn a_loop_resistor_is_dropped() {
        let net = Network { node_caps: vec![0.0, 1e-15, 1e-15], resistors: vec![(0, 1, 10.0), (1, 2, 10.0), (2, 0, 10.0), (1, 1, 5.0)] };
        let pins = NodePins { pin_cap: &|_| 0.0, is_load: &|n| n == 2 };
        let with_loop = reduce_to_pi_elmore(&net, 0, &pins);
        let tree = Network { node_caps: vec![0.0, 1e-15, 1e-15], resistors: vec![(0, 1, 10.0), (1, 2, 10.0)] };
        assert_eq!(with_loop, reduce_to_pi_elmore(&tree, 0, &pins));
    }
}
