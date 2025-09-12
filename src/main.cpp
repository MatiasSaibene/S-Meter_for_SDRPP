#include "main.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <imgui.h>
#include <sdrpp_core/src/config.h>
#include <sdrpp_core/src/imgui/imgui.h>
#include <sdrpp_core/src/imgui/imgui_internal.h>
#include <sdrpp_core/src/utils/flog.h>
#include <sdrpp_core/src/core.h>
#include <sdrpp_core/src/gui/gui.h>
#include <sdrpp_core/src/json.hpp>
#include <sdrpp_core/src/module.h>
#include <sdrpp_core/src/signal_path/signal_path.h>
#include <string>

#define GImGui (ImGui::GetCurrentContext())

SDRPP_MOD_INFO {
    /* Name:            */ "smeter_plugin",
    /* Description:     */ "Signal meter (S-Meter) plugin for SDR++",
    /* Author:          */ "MatiasSaibene",
    /* Version:         */ 0, 1, 0,
    /* Max instances    */ 1
};

ModuleManager mdlmgr;
ConfigManager configmgr;

SMeterModule::SMeterModule(std::string name){

    this->name = name;
    
    gui::menu.registerEntry(name, menuHandler, this, NULL);


}

SMeterModule::~SMeterModule(){
    if(isEnabled()){
        disable();
    }
    gui::menu.removeEntry(name);
    if(fftHandler){
        gui::waterfall.onFFTRedraw.unbindHandler(fftHandler);
        delete fftHandler;
    }
}

void SMeterModule::UpdateSMeterAngle(){

    if(!wtf){
        flog::__log__(flog::TYPE_WARNING, "SMeterModule: waterfall pointer is null", {});
        return;
    }

    if(!wtf->selectedVFO.empty()){
        auto it = wtf->vfos.find(wtf->selectedVFO);
        if(it != wtf->vfos.end()){
            myVFO = it->second;
        }
    }

    if(!myVFO){
        flog::__log__(flog::TYPE_DEBUG, "SMeterModule: no VFO selected", {});
        return;
    }



    int fftWidth = 0;

    float *fftLine = wtf->acquireLatestFFT(fftWidth);
    
    if(!fftLine || fftWidth <= 0){
        if(fftLine){
            wtf->releaseLatestFFT();
            flog::__log__(flog::TYPE_DEBUG, "SMeterModule: no FFT available", {});
            return;
        }
    }


    wholeBandwidth = wtf->getBandwidth();
    int rawFFTSize = fftWidth;

    if(rawFFTSize < 2){
        wtf->releaseLatestFFT();
        return;
    }

    float local_strength = 0.0f;
    float local_snr = 0.0f;

    bool ok = calculateVFOSignalInfo(fftLine, myVFO, 1, wholeBandwidth, rawFFTSize, local_strength, local_snr);

    if(!ok){
        flog::__log__(flog::TYPE_DEBUG, "SMeterModule: calculateVFOSignalInfo returned false", {});
    }

    strength = local_strength;
    snr = local_snr;

    wtf->releaseLatestFFT();
}

bool SMeterModule::calculateVFOSignalInfo(float *fftLine, ImGui::WaterfallVFO *wtfvfo, int fftLines, double wholeBandwidth, int rawFFTSize, float &strength, float &snr){

    if (fftLine == nullptr || fftLines <= 0 || rawFFTSize <= 0 || wtfvfo == nullptr) {
        return false;
    }

    // Compute bins offsets
    double vfoMinSizeFreq = wtfvfo->centerOffset - wtfvfo->bandwidth;
    double vfoMinFreq = wtfvfo->centerOffset - (wtfvfo->bandwidth / 2.0);
    double vfoMaxFreq = wtfvfo->centerOffset + (wtfvfo->bandwidth / 2.0);
    double vfoMaxSizeFreq = wtfvfo->centerOffset + wtfvfo->bandwidth;

    //Avoid division by 0
    if (wholeBandwidth <= 0.0) return false;

    //Frequency mapping -> bin (FFT centered in rawFFTSize/2), clamp up to rawFFTSize-1
    auto freqToBin = [&](double freq)->int {
        double ratio = (freq / (wholeBandwidth / 2.0)) * (double)(rawFFTSize / 2.0);
        int idx = static_cast<int>(std::round(ratio + (rawFFTSize / 2.0)));
        if (idx < 0) idx = 0;
        if (idx > rawFFTSize - 1) idx = rawFFTSize - 1;
        return idx;
    };

    int vfoMinSideOffset = freqToBin(vfoMinSizeFreq);
    int vfoMinOffset     = freqToBin(vfoMinFreq);
    int vfoMaxOffset     = freqToBin(vfoMaxFreq);
    int vfoMaxSideOffset = freqToBin(vfoMaxSizeFreq);

    //Ensure order
    if (vfoMinSideOffset > vfoMinOffset) std::swap(vfoMinSideOffset, vfoMinOffset);
    if (vfoMaxOffset > vfoMaxSideOffset) std::swap(vfoMaxOffset, vfoMaxSideOffset);

    double avg = 0.0;
    float maxVal = -INFINITY;
    int avgCount = 0;

    //Left average
    if (vfoMinSideOffset < vfoMinOffset) {
        for (int i = vfoMinSideOffset; i < vfoMinOffset; ++i) {
            avg += static_cast<double>(fftLine[i]);
            ++avgCount;
        }
    }

    //Right average
    if (vfoMaxOffset + 1 <= vfoMaxSideOffset - 1) {
        for (int i = vfoMaxOffset + 1; i < vfoMaxSideOffset; ++i) {
            avg += static_cast<double>(fftLine[i]);
            ++avgCount;
        }
    }

    if (avgCount == 0) {
        //if no valid lateral bin -> SNR calculation impossible
        return false;
    }

    avg /= static_cast<double>(avgCount);

    //Find signal max
    if (vfoMinOffset <= vfoMaxOffset) {
        for (int i = vfoMinOffset; i <= vfoMaxOffset; ++i) {
            if (fftLine[i] > maxVal) {
                maxVal = fftLine[i];
            }
        }
    } else {
        //if invalind range
        return false;
    }

    strength = maxVal;
    snr = maxVal - static_cast<float>(avg);

    return true;
}

void SMeterModule::DrawSMeter(float norm)
{
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    ImVec2 p = ImGui::GetCursorScreenPos();
    float size = 200.0f;                //widget size
    ImVec2 center = ImVec2(p.x + size * 0.5f, p.y + size * 0.75f);
    float radius = size * 0.45f;

    ImU32 col_arc    = IM_COL32(255, 255, 255, 255);
    ImU32 col_red    = IM_COL32(255,   0,   0, 255);
    ImU32 col_needle = IM_COL32(255, 255, 255, 255);

    //Base arc
    float angle_min = 5.0f * IM_PI / 4.0f; // 225°
    float angle_max = 7.0f * IM_PI / 4.0f; // 315°

    //White arc
    draw_list->PathArcTo(center, radius, angle_min, angle_max, 64);
    draw_list->PathStroke(col_arc, 0, 3.0f);

    //Red arc
    float angle_red_start = angle_min + (angle_max - angle_min) * 0.75f;
    draw_list->PathArcTo(center, radius, angle_red_start, angle_max, 32);
    draw_list->PathStroke(col_red, 0, 3.0f);

    //Needle
    float angle = angle_min + (angle_max - angle_min) * norm;
    ImVec2 needle(
        center.x + cosf(angle) * (radius - 15),
        center.y + sinf(angle) * (radius - 15)   // 👈 cambio: +sinf
    );
    draw_list->AddLine(center, needle, col_needle, 3.0f);

    // Ticks + labels
    const char* labels[] = { "S1","S3","S5","S7","S9","+10","+30","+60" };
    const int ticks = 7;

    for (int i = 0; i <= ticks; i++) {
        float t = (float)i / (float)ticks;
        float a = angle_min + (angle_max - angle_min) * t;

        ImVec2 p1(
            center.x + cosf(a) * (radius - 8),
            center.y + sinf(a) * (radius - 8)
        );
        ImVec2 p2(
            center.x + cosf(a) * (radius - 2),
            center.y + sinf(a) * (radius - 2)
        );
        draw_list->AddLine(p1, p2, col_arc, 2.0f);

        if (i < IM_ARRAYSIZE(labels)) {
            ImVec2 text_size = ImGui::CalcTextSize(labels[i]);
            ImVec2 text_pos(
                center.x + cosf(a) * (radius + 12) - text_size.x * 0.5f,
                center.y + sinf(a) * (radius + 12) - text_size.y * 0.5f
            );
            draw_list->AddText(text_pos, col_arc, labels[i]);
        }
    }
}


bool SMeterModule::isEnabled(){
    return enabled;
}

void SMeterModule::postInit(){

    wtf = &gui::waterfall;
    if (!wtf) {
        flog::__log__(flog::TYPE_ERROR, "SMeterModule: gui::waterfall not available", {});
        return;
    }

    if(!wtf->selectedVFO.empty()){
        auto it = wtf->vfos.find(wtf->selectedVFO);
        if(it != wtf->vfos.end()){
            myVFO = it->second;
        } else {
            myVFO = nullptr;
        }
    }

    fftHandler = new EventHandler<ImGui::WaterFall::FFTRedrawArgs>([](ImGui::WaterFall::FFTRedrawArgs args, void *ctx){
        auto *self = static_cast<SMeterModule *>(ctx);
        if(self->enabled){
            self->UpdateSMeterAngle();
        }
    },
    this
    );
    
    wtf->onFFTRedraw.bindHandler(fftHandler);
    flog::info("SMeterModule: handler bound to Waterfall FFT redraw");

}

void SMeterModule::enable(){
    enabled = true;
}

void SMeterModule::disable(){
    enabled = false;
}

void SMeterModule::menuHandler(void *ctx){

    SMeterModule * _this = (SMeterModule*)ctx;

    if(!_this->isEnabled()){
        return;
    }

    if(ImGui::CollapsingHeader("S-Meter")){

        //Take all width available
        ImVec2 avail = ImGui::GetContentRegionAvail();

        //Adjust proportion = 50%
        ImVec2 size(avail.x, avail.x * 0.5f);

        ImGui::BeginChild("S-MeterWidget", size, false, ImGuiWindowFlags_NoScrollbar);

        _this->dBFS = _this->strength;
        _this->S9_dBFS = -35.0f;      //calibrate this!!!

        //Compute S-Units
        float s_units = 9.0f + (_this->dBFS - _this->S9_dBFS) / 6.0f;

        //Normalize (S1 to S9+60 ≈ 16.6 units)
        float norm = std::clamp(s_units / 16.6f, 0.0f, 1.0f);

        _this->DrawSMeter(norm);

        ImGui::EndChild();
        
        ImGui::Text("Level: S%.1f", s_units);
        ImGui::Text("Signal strength: %.1fdBFS", _this->dBFS);
        ImGui::Text("Signal/Noise Ratio: %.1fdB", _this->snr);
    }
    
}

MOD_EXPORT void _INIT_(){

    std::string root = (std::string)core::args["root"];
    json def = json({});
    configmgr.setPath(root + "/smeter_plugin_config.json");
    configmgr.load(def);
    configmgr.enableAutoSave();

}

MOD_EXPORT SMeterModule::Instance *_CREATE_INSTANCE_(std::string name){
    return new SMeterModule(name);
}

MOD_EXPORT void _DELETE_INSTANCE_(void *instance){
    delete (SMeterModule *)instance;
}

MOD_EXPORT void _END_(){
    
    configmgr.disableAutoSave();
    configmgr.save();
}