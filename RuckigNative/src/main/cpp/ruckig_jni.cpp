#include <jni.h>

#include <ruckig/ruckig.hpp>

using namespace ruckig;

// Aggregate that keeps the Ruckig planner together with its reusable I/O
// parameter objects so we avoid heap-allocating them on every update() call.
struct RuckigHandle {
    int dofs;
    Ruckig<DynamicDOFs> ruckig;
    InputParameter<DynamicDOFs>  input;
    OutputParameter<DynamicDOFs> output;

    RuckigHandle(int dofs, double cycleTime)
        : dofs(dofs),
          ruckig(dofs, cycleTime),
          input(dofs),
          output(dofs) {}
};

extern "C" {

// org.marsroboticsassociation.controllib.ruckig.RuckigController.nativeCreate
JNIEXPORT jlong JNICALL
Java_org_marsroboticsassociation_controllib_ruckig_RuckigController_nativeCreate(
        JNIEnv* /*env*/, jobject /*thiz*/, jint dofs, jdouble cycleTime) {
    auto* handle = new RuckigHandle(static_cast<int>(dofs),
                                    static_cast<double>(cycleTime));
    return reinterpret_cast<jlong>(handle);
}

// org.marsroboticsassociation.controllib.ruckig.RuckigController.nativeUpdate
JNIEXPORT jint JNICALL
Java_org_marsroboticsassociation_controllib_ruckig_RuckigController_nativeUpdate(
        JNIEnv* env, jobject /*thiz*/, jlong handlePtr,
        jdoubleArray currentPos, jdoubleArray currentVel, jdoubleArray currentAcc,
        jdoubleArray targetPos,  jdoubleArray targetVel,  jdoubleArray targetAcc,
        jdoubleArray maxVel,     jdoubleArray maxAcc,     jdoubleArray maxJerk,
        jdoubleArray outPos,     jdoubleArray outVel,     jdoubleArray outAcc,
        jdoubleArray outDuration) {

    auto* h   = reinterpret_cast<RuckigHandle*>(handlePtr);
    int   dofs = h->dofs;
    auto& inp  = h->input;
    auto& out  = h->output;

    // Helper: copy a Java double[] into a std::vector<double>.
    // We resize in-place to avoid reallocation on every call.
    auto load = [&](jdoubleArray arr, std::vector<double>& vec) {
        vec.resize(dofs);
        env->GetDoubleArrayRegion(arr, 0, dofs, vec.data());
    };

    load(currentPos, inp.current_position);
    load(currentVel, inp.current_velocity);
    load(currentAcc, inp.current_acceleration);
    load(targetPos,  inp.target_position);
    load(targetVel,  inp.target_velocity);
    load(targetAcc,  inp.target_acceleration);
    load(maxVel,     inp.max_velocity);
    load(maxAcc,     inp.max_acceleration);
    load(maxJerk,    inp.max_jerk);

    Result result = h->ruckig.update(inp, out);

    // Write kinematic output back to Java arrays.
    env->SetDoubleArrayRegion(outPos, 0, dofs, out.new_position.data());
    env->SetDoubleArrayRegion(outVel, 0, dofs, out.new_velocity.data());
    env->SetDoubleArrayRegion(outAcc, 0, dofs, out.new_acceleration.data());

    // outDuration[0] = total planned trajectory duration.
    double dur = out.trajectory.get_duration();
    env->SetDoubleArrayRegion(outDuration, 0, 1, &dur);

    return static_cast<jint>(result);
}

// org.marsroboticsassociation.controllib.ruckig.RuckigController.nativeDestroy
JNIEXPORT void JNICALL
Java_org_marsroboticsassociation_controllib_ruckig_RuckigController_nativeDestroy(
        JNIEnv* /*env*/, jobject /*thiz*/, jlong handlePtr) {
    delete reinterpret_cast<RuckigHandle*>(handlePtr);
}

} // extern "C"
