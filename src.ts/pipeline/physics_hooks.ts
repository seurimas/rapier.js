import { RawVector } from "../raw";
import { RigidBodyHandle } from "../dynamics";
import { ColliderHandle } from "../geometry";
import { Vector } from "../math";

export enum ActiveHooks {
    NONE = 0,
    FILTER_CONTACT_PAIRS = 0b0001,
    FILTER_INTERSECTION_PAIRS = 0b0010,
    MODIFY_SOLVER_CONTACTS = 0b0100,
}

export enum SolverFlags {
    EMPTY = 0b000,
    COMPUTE_IMPULSE = 0b001,
}

export declare class ContactModificationContext {
    collider1: ColliderHandle;
    collider2: ColliderHandle;
    body1: RigidBodyHandle;
    body2: RigidBodyHandle;

    normal: RawVector;
    user_data: number;

    num_solver_contacts(): number;

    clear_solver_contacts(): void;

    remove_solver_contact(index: number): void;

    solver_contact_point(index: number): RawVector | null;
    set_solver_contact_point(index: number, point: RawVector): void;

    solver_contact_dist(index: number): number;
    set_solver_contact_dist(index: number, dist: number): void;

    solver_contact_friction(index: number): number;
    set_solver_contact_friction(index: number, friction: number): void;

    solver_contact_restitution(index: number): number;
    set_solver_contact_restitution(index: number, restitution: number): void;

    solver_contact_tangent_velocity(index: number): RawVector | null;
    set_solver_contact_tangent_velocity(
        index: number,
        tangent_velocity: RawVector,
    ): void;

    solver_contact_warmstart_impulse(index: number): number;
    set_solver_contact_warmstart_impulse(index: number, impulse: number): void;

    solver_contact_warmstart_tangent_impulse(index: number): number;
    set_solver_contact_warmstart_tangent_impulse(
        index: number,
        impulse: number,
    ): void;

    solver_contact_warmstart_twist_impulse(index: number): number;
    set_solver_contact_warmstart_twist_impulse(
        index: number,
        impulse: number,
    ): void;

    solver_contact_is_new(index: number): boolean;
    set_solver_contact_is_new(index: number, is_new: boolean): void;
}

export interface PhysicsHooks {
    /**
     * Function that determines if contacts computation should happen between two colliders, and how the
     * constraints solver should behave for these contacts.
     *
     * This will only be executed and taken into account if at least one of the involved colliders contains the
     * `ActiveHooks.FILTER_CONTACT_PAIR` flag in its active hooks.
     *
     * @param collider1 − Handle of the first collider involved in the potential contact.
     * @param collider2 − Handle of the second collider involved in the potential contact.
     * @param body1 − Handle of the first body involved in the potential contact.
     * @param body2 − Handle of the second body involved in the potential contact.
     */
    filterContactPair(
        collider1: ColliderHandle,
        collider2: ColliderHandle,
        body1: RigidBodyHandle,
        body2: RigidBodyHandle,
    ): SolverFlags | null;

    /**
     * Function that determines if intersection computation should happen between two colliders (where at least
     * one is a sensor).
     *
     * This will only be executed and taken into account if `one of the involved colliders contains the
     * `ActiveHooks.FILTER_INTERSECTION_PAIR` flag in its active hooks.
     *
     * @param collider1 − Handle of the first collider involved in the potential contact.
     * @param collider2 − Handle of the second collider involved in the potential contact.
     * @param body1 − Handle of the first body involved in the potential contact.
     * @param body2 − Handle of the second body involved in the potential contact.
     */
    filterIntersectionPair(
        collider1: ColliderHandle,
        collider2: ColliderHandle,
        body1: RigidBodyHandle,
        body2: RigidBodyHandle,
    ): boolean;

    /**
     * Function that modifies the set of contacts seen by the constraints solver.
     *
     * Note that this method will only be called if at least one of the colliders
     * involved in the contact contains the `ActiveHooks::MODIFY_SOLVER_CONTACTS` flags
     * in its physics hooks flags.
     *
     * @param context - The context providing information and access to the contacts to modify.
     */
    modifySolverContacts(context: ContactModificationContext): void;
}
