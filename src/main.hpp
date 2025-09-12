#pragma once
#ifndef __MAIN_HPP_
#define __MAIN_HPP_

#include <sdrpp_core/src/gui/widgets/waterfall.h>
#include <sdrpp_core/src/signal_path/vfo_manager.h>
#include <string>
#include <sdrpp_core/src/imgui/imgui.h>
#include <sdrpp_core/src/core.h>
#include <sdrpp_core/src/gui/widgets/volume_meter.h>
#include <sdrpp_core/src/module.h>

class SMeterModule : public ModuleManager::Instance{

    public:
        
        SMeterModule(std::string name);
        
        ~SMeterModule();

        bool calculateVFOSignalInfo(float *fftLine, ImGui::WaterfallVFO *wtfvfo, int fftLines, double wholeBandwidth, int rawFFTSize, float &strength, float &snr);

        void UpdateSMeterAngle();

        void DrawSMeter(float strengthNorm);

        static void menuHandler(void *ctx);

        void postInit() override;

        void enable() override;

        void disable() override;

        bool isEnabled() override;

    private:

        std::string name;

        bool enabled = true;

        double wholeBandwidth;

        float strength, snr, dBFS, S9_dBFS;

        ImGui::WaterfallVFO *myVFO;
        ImGui::WaterFall *wtf;

        struct VFOSignalInfo{
            float strength;
            float snr;
        };

        EventHandler<ImGui::WaterFall::FFTRedrawArgs>* fftHandler = nullptr;

};

#endif //!__MAIN_HPP_