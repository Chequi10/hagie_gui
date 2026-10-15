#include "ai/onnx_tassel_detector.h"
#include <QCoreApplication>
#include <QFile>
#include <QTemporaryDir>
#include <cassert>

int main(int argc, char** argv) {
    QCoreApplication app(argc, argv);
    QTemporaryDir temp;
    QFile model(temp.filePath("model.onnx"));
    assert(model.open(QIODevice::WriteOnly)); model.write("test"); model.close();
    QFile helper(temp.filePath("detector.py"));
    assert(helper.open(QIODevice::WriteOnly));
    helper.write(R"PY(import base64, json, sys
print(json.dumps({'type':'ready'}), flush=True)
for line in sys.stdin:
    request=json.loads(line)
    assert request['width']==3 and request['height']==1
    assert base64.b64decode(request['rgb'])==bytes([1,2,3,4,5,6,7,8,9])
    assert abs(request['iou']-0.45)<0.0001
    print(json.dumps({'type':'result','timestamp_ms':request['timestamp_ms'],
          'boxes':[[0,0,3,1,request['confidence']]],'inference_ms':12}), flush=True)
)PY");
    helper.close();
    std::atomic<bool> running {true};
    OnnxTasselDetector detector;
    assert(detector.initialize("/usr/bin/python3", helper.fileName().toStdString(), model.fileName().toStdString(), running));
    RgbFrameSource::Frame frame;
    frame.width=3; frame.height=1; frame.data={1,2,3,4,5,6,7,8,9};
    frame.timestamp_ms=123; frame.valid=true; frame.rgb_only=true;
    TasselDetector::Result result;
    assert(detector.processFrame(0,frame,result,0.1f,0.45f,running));
    assert(result.valid && result.timestamp_ms==123 && result.detections.size()==1);
    assert(result.detections[0].confidence==0.1f && !result.detections[0].position_3d_valid);
    assert(result.detections[0].width==3 && detector.inferenceMs()==12);
    frame.data.pop_back();
    assert(!detector.processFrame(0,frame,result,0.1f,0.45f,running));
}
