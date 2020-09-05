pub use nalgebra as na;

macro_rules! make_explore_around {
    ($argtype:ty) => {
        fn explore_around(
            x: $argtype, per: f64, 
            f: impl Fn($argtype) -> f64
        ) -> Option<$argtype> {
        
            let mut nextx = x;
            let mut fbest = f(x);
            let mut changed = false;
            for i in 0..nextx.len() {
                nextx[i] += per;
                let fnext = f(nextx);
                if fnext < fbest {
                    fbest = fnext;
                    changed = true;
                    continue;
                } else {
                    nextx[i] -= 2.0 * per;
                    let fnext = f(nextx);
                    if fnext < fbest {
                        fbest = fnext;
                        changed = true;
                    } else {
                        nextx[i] = x[i];
                    }
                }
            }
        
            if changed {
                Some(nextx)
            } else {
                None
            }
        }
    };
}

macro_rules! make_pattern_move {
    ($argtype:ty) => {
        fn pattern_move(
            x: &mut $argtype, mut nextx: $argtype, 
            per: f64, f: impl Fn($argtype) -> f64
        ) {
            loop {
                let mut farx = 2.0 * nextx - *x;
                if let Some(nextfarx) = explore_around(farx, per, |x| f(x)) {
                    farx = nextfarx;
                }
        
                if f(farx) >= f(*x) {
                    *x = nextx;
                    break;
                } else {
                    *x = nextx;
                    nextx = farx;
                }
            }
        }
    };
}

macro_rules! make_search_fn {
    ($name:ident, $argtype:ty) => {
        pub fn $name(
            init_arg: $argtype, init_per: f64, eps: f64, 
            f: impl Fn($argtype) -> f64
        ) -> $argtype {
        
            assert!(init_per > 0.0 && eps > 0.0);

            make_explore_around!($argtype);
            make_pattern_move!($argtype);
        
            let mut x = init_arg;
            let mut per = init_per;
            while per > eps {
                if let Some(nextx) = explore_around(x, per, |x| f(x)) {
                    per = init_per;
                    pattern_move(&mut x, nextx, per, |x| f(x));
                } else {
                    per *= 0.5;
                }
            }
            
            x
        }
    };
}

macro_rules! make_search_with_n_fn {
    ($name:ident, $argtype:ty) => {
        pub fn $name(
            init_arg: $argtype, init_per: f64, n: usize, 
            f: impl Fn($argtype) -> f64
        ) -> $argtype {
        
            assert!(init_per > 0.0 && n > 0);

            make_explore_around!($argtype);
            make_pattern_move!($argtype);
        
            let mut x = init_arg;
            let mut per = init_per;
            for _ in 0..n {
                if let Some(nextx) = explore_around(x, per, |x| f(x)) {
                    per = init_per;
                    pattern_move(&mut x, nextx, per, |x| f(x));
                } else {
                    per *= 0.5;
                }
            }
            
            x
        }
    };
}

macro_rules! make_search_with_penalty_fn {
    ($name:ident, $argtype:ty) => {
        pub fn $name(
            init_arg: $argtype, init_per: f64, eps: f64, 
            f: impl Fn($argtype, f64) -> f64,
            coef: impl Fn(usize) -> f64,
        ) -> $argtype {
        
            assert!(init_per > 0.0 && eps > 0.0);

            make_explore_around!($argtype);
            make_pattern_move!($argtype);
        
            let mut x = init_arg;
            let mut per = init_per;
            let mut count = 1;
            while per > eps {
                if let Some(nextx) = explore_around(x, per, |x| f(x, coef(count))) {
                    per = init_per;
                    pattern_move(&mut x, nextx, per, |x| f(x, coef(count)));
                } else {
                    per *= 0.5;
                }
                count += 1;
            }
            
            x
        }
    };
}

macro_rules! make_search_with_penalty_and_n_fn {
    ($name:ident, $argtype:ty) => {
        pub fn $name(
            init_arg: $argtype, init_per: f64, n: usize, 
            f: impl Fn($argtype, f64) -> f64,
            coef: impl Fn(usize) -> f64,
        ) -> $argtype {
        
            assert!(init_per > 0.0 && n > 0);

            make_explore_around!($argtype);
            make_pattern_move!($argtype);
        
            let mut x = init_arg;
            let mut per = init_per;
            for i in 0..n {
                if let Some(nextx) = explore_around(x, per, |x| f(x, coef(i + 1))) {
                    per = init_per;
                    pattern_move(&mut x, nextx, per, |x| f(x, coef(i + 1)));
                } else {
                    per *= 0.5;
                }
            }
            
            x
        }
    };
}



make_search_fn!(search_2d, na::Vector2<f64>);
make_search_fn!(search_3d, na::Vector3<f64>);
make_search_with_n_fn!(search_with_n_2d, na::Vector2<f64>);
make_search_with_n_fn!(search_with_n_3d, na::Vector3<f64>);
make_search_with_penalty_fn!(search_with_penalty_2d, na::Vector2<f64>);
make_search_with_penalty_fn!(search_with_penalty_3d, na::Vector3<f64>);
make_search_with_penalty_and_n_fn!(search_with_penalty_and_n_2d, na::Vector2<f64>);
make_search_with_penalty_and_n_fn!(search_with_penalty_and_n_3d, na::Vector3<f64>);
