use crate::math::RawVector;
use crate::utils::{self, FlatHandle};
use rapier::geometry::SolverFlags;
use rapier::math::{Real, Vector};
use rapier::pipeline::{ContactModificationContext, PairFilterContext, PhysicsHooks};
use rapier::prelude::{ContactManifold, SolverContact};
use wasm_bindgen::prelude::*;

pub struct RawPhysicsHooks {
    pub this: js_sys::Object,
    pub filter_contact_pair: js_sys::Function,
    pub filter_intersection_pair: js_sys::Function,
    pub modify_solver_contacts: js_sys::Function,
}

// HACK: the RawPhysicsHooks is no longer Send+Sync because the JS objects are
//       no longer Send+Sync since https://github.com/rustwasm/wasm-bindgen/pull/955
//       As far as this is confined to the bindings this should be fine since we
//       never use threading in wasm.
unsafe impl Send for RawPhysicsHooks {}
unsafe impl Sync for RawPhysicsHooks {}

#[wasm_bindgen]
extern "C" {
    // Use `js_namespace` here to bind `console.log(..)` instead of just
    // `log(..)`
    #[wasm_bindgen(js_namespace = console)]
    fn log(s: &str);
}

impl PhysicsHooks for RawPhysicsHooks {
    fn filter_contact_pair(&self, ctxt: &PairFilterContext) -> Option<SolverFlags> {
        let rb1 = ctxt
            .rigid_body1
            .map(|rb| JsValue::from(utils::flat_handle(rb.0)))
            .unwrap_or(JsValue::NULL);
        let rb2 = ctxt
            .rigid_body2
            .map(|rb| JsValue::from(utils::flat_handle(rb.0)))
            .unwrap_or(JsValue::NULL);

        let result = self
            .filter_contact_pair
            .bind2(
                &self.this,
                &JsValue::from(utils::flat_handle(ctxt.collider1.0)),
                &JsValue::from(utils::flat_handle(ctxt.collider2.0)),
            )
            .call2(&self.this, &rb1, &rb2)
            .ok()?;
        let flags = result.as_f64()?;
        // TODO: not sure exactly why we have to do `flags as u32` instead
        //       of `flags.to_bits() as u32`.
        SolverFlags::from_bits(flags as u32)
    }

    fn filter_intersection_pair(&self, ctxt: &PairFilterContext) -> bool {
        let rb1 = ctxt
            .rigid_body1
            .map(|rb| JsValue::from(utils::flat_handle(rb.0)))
            .unwrap_or(JsValue::NULL);
        let rb2 = ctxt
            .rigid_body2
            .map(|rb| JsValue::from(utils::flat_handle(rb.0)))
            .unwrap_or(JsValue::NULL);

        self.filter_intersection_pair
            .bind2(
                &self.this,
                &JsValue::from(utils::flat_handle(ctxt.collider1.0)),
                &JsValue::from(utils::flat_handle(ctxt.collider2.0)),
            )
            .call2(&self.this, &rb1, &rb2)
            .ok()
            .and_then(|res| res.as_bool())
            .unwrap_or(false)
    }

    fn modify_solver_contacts(&self, ctxt: &mut ContactModificationContext) {
        let raw_context = RawContactModificationContext {
            collider1: utils::flat_handle(ctxt.collider1.0),
            collider2: utils::flat_handle(ctxt.collider2.0),
            rigid_body1: ctxt.rigid_body1.map(|rb| utils::flat_handle(rb.0)),
            rigid_body2: ctxt.rigid_body2.map(|rb| utils::flat_handle(rb.0)),
            manifold: ctxt.manifold as *const ContactManifold,
            solver_contacts: ctxt.solver_contacts as *mut Vec<SolverContact>,
            normal: ctxt.normal as *mut Vector<Real>,
            user_data: ctxt.user_data as *mut u32,
        };
        let _ = self
            .modify_solver_contacts
            .call1(&self.this, &JsValue::from(raw_context));
    }
}

#[wasm_bindgen]
pub struct RawContactModificationContext {
    pub collider1: FlatHandle,
    pub collider2: FlatHandle,
    pub rigid_body1: Option<FlatHandle>,
    pub rigid_body2: Option<FlatHandle>,
    pub manifold: *const ContactManifold,
    pub solver_contacts: *mut Vec<SolverContact>,
    normal: *mut Vector<Real>,
    user_data: *mut u32,
}

#[wasm_bindgen]
impl RawContactModificationContext {
    pub fn collider1(&self) -> FlatHandle {
        self.collider1
    }

    pub fn collider2(&self) -> FlatHandle {
        self.collider2
    }

    #[wasm_bindgen(getter)]
    pub fn normal(&self) -> RawVector {
        unsafe { RawVector(*self.normal) }
    }

    #[wasm_bindgen(setter)]
    pub fn set_normal(&mut self, normal: RawVector) {
        unsafe {
            *self.normal = normal.0;
        }
    }

    #[wasm_bindgen(getter)]
    pub fn user_data(&self) -> u32 {
        unsafe { *self.user_data }
    }

    #[wasm_bindgen(setter)]
    pub fn set_user_data(&mut self, user_data: u32) {
        unsafe {
            *self.user_data = user_data;
        }
    }

    pub fn num_solver_contacts(&self) -> usize {
        unsafe { (*self.solver_contacts).len() }
    }

    pub fn clear_solver_contacts(&mut self) {
        unsafe { (*self.solver_contacts).clear() }
    }

    pub fn remove_solver_contact(&mut self, i: usize) {
        unsafe {
            if i < self.num_solver_contacts() {
                (*self.solver_contacts).swap_remove(i);
            }
        }
    }

    pub fn solver_contact_point(&self, i: usize) -> Option<RawVector> {
        unsafe {
            (&(*self.solver_contacts))
                .get(i)
                .map(|c| c.point.coords.into())
        }
    }

    pub fn set_solver_contact_point(&mut self, i: usize, pt: &RawVector) {
        unsafe {
            if let Some(c) = (&mut (*self.solver_contacts)).get_mut(i) {
                c.point = pt.0.into()
            }
        }
    }

    pub fn solver_contact_dist(&self, i: usize) -> Real {
        unsafe {
            (&(*self.solver_contacts))
                .get(i)
                .map(|c| c.dist)
                .unwrap_or(0.0)
        }
    }

    pub fn set_solver_contact_dist(&mut self, i: usize, dist: Real) {
        unsafe {
            if let Some(c) = (&mut (*self.solver_contacts)).get_mut(i) {
                c.dist = dist
            }
        }
    }

    pub fn solver_contact_friction(&self, i: usize) -> Real {
        unsafe { (&(*self.solver_contacts))[i].friction }
    }

    pub fn set_solver_contact_friction(&mut self, i: usize, friction: Real) {
        unsafe {
            if let Some(c) = (&mut (*self.solver_contacts)).get_mut(i) {
                c.friction = friction
            }
        }
    }

    pub fn solver_contact_restitution(&self, i: usize) -> Real {
        unsafe { (&(*self.solver_contacts))[i].restitution }
    }

    pub fn set_solver_contact_restitution(&mut self, i: usize, restitution: Real) {
        unsafe {
            if let Some(c) = (&mut (*self.solver_contacts)).get_mut(i) {
                c.restitution = restitution
            }
        }
    }

    pub fn solver_contact_tangent_velocity(&self, i: usize) -> RawVector {
        unsafe { (&(*self.solver_contacts))[i].tangent_velocity.into() }
    }

    pub fn set_solver_contact_tangent_velocity(&mut self, i: usize, vel: &RawVector) {
        unsafe {
            if let Some(c) = (&mut (*self.solver_contacts)).get_mut(i) {
                c.tangent_velocity = vel.0.into()
            }
        }
    }
}
