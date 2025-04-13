#include <cassert>

#include <QTimer>
#include <QFileDialog>

#include "qcustomplot.h"
#include "LiveChart.h"

bool LiveChart::paused = false;
QVector<LiveChart *> LiveChart::registered;

LiveChart::LiveChart(const Config &config, QWidget *parent) : QCustomPlot{parent} {
    registered.append(this);
    title = config.title;

    QSharedPointer<QCPAxisTickerTime> xTicker(new QCPAxisTickerTime);
    xTicker->setTimeFormat("%m:%s");
    xAxis->setTicker(xTicker);
    QSharedPointer<QCPAxisTickerFixed> yTicker(new QCPAxisTickerFixed);
    yTicker->setTickStep(config.yTick);
    yAxis->setTicker(yTicker);
    yAxis->setRange(config.yMin, config.yMax);
    yAxis->setLabel(config.yLabel);
    yAxis->setNumberFormat("f");
    yAxis->setNumberPrecision(config.yPrecision);

    plotLayout()->insertRow(0);
    QCPTextElement *title = new QCPTextElement(this, config.title, QFont());
    title->setAutoMargins(QCP::msNone);
    title->setMargins(QMargins(0, 10, 0, -15));
    plotLayout()->addElement(0, 0, title);

    setMinimumSize(400, 200);
    setBackground(Qt::transparent);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setOpenGl(true);

    QTimer *timer = new QTimer();
    connect(timer, &QTimer::timeout, [this]() {
        if(!paused) {
            replot();
        }
    });
    timer->start(20);
}

void LiveChart::addSeries(const QString name, const QPen pen) {
    addGraph();
    series.append(name);

    const int index = series.indexOf(name);

    graph(index)->setPen(pen);
    graph(index)->setAdaptiveSampling(true);
}

void LiveChart::append(const QString name, const double time, const double value) {
    if(!paused) {
        assert(series.contains(name));

        const int index = series.indexOf(name);

        graph(index)->addData(time, value);
    }
}

void LiveChart::synchronize(const double time) {
    for(LiveChart *chart : registered) {
        for(int i=0; i<chart->series.size(); i++) {
            chart->graph(i)->data()->removeBefore(time-10);
            chart->graph(i)->data()->removeAfter(time);
        }
        chart->xAxis->setRange(time, 10, Qt::AlignRight);
    }
}

void LiveChart::resume() {
    paused = false;

    for(LiveChart *chart : registered) {
        for(int i=0; i<chart->series.size(); i++) {
            chart->graph(i)->data()->clear();
        }
    }
}

void LiveChart::pause() {
    paused = true;
}

void LiveChart::save() {
    pause();

    QString basename = "livechart_" + QDateTime::currentDateTime().toString("yyyy-MM-dd_HH:mm:ss");

    basename = QFileDialog::getSaveFileName(nullptr, "DialogTitle", basename).replace(" ", "_");

    if(basename.isEmpty()) {
        return;
    }

    QDir().mkdir(basename);

    for(const LiveChart *chart : registered) {
        for(int i=0; i<chart->series.size(); i++) {
            QString name = chart->title + "_" + chart->series[i];

            QFile file(basename + "/" + name.replace(" ", "_") + ".csv");

            if(file.open(QFile::WriteOnly | QFile::Text)) {
                QTextStream output(&file);

                output << "time,value\n";

                const QSharedPointer<QCPGraphDataContainer> data = chart->graph(i)->data();
                for(const QCPGraphData &point : *data) {
                    output << point.key << "," << point.value << "\n";
                }

                file.close();
            }
        }
    }
}
