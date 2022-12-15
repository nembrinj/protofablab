import { IfStmt } from '@angular/compiler';
import { Component, OnInit } from '@angular/core';
import { environment } from 'src/environments/environment';

@Component({
  selector: 'app-config',
  templateUrl: './config.component.html',
  styleUrls: ['./config.component.sass']
})
export class ConfigComponent implements OnInit {

  sensorConfig = {
    amp: 50,
    duration: 500,
    sample_rate: 1000,
    drop_rate: 1000
  }
  configChanged = false

  constructor() { }

  async ngOnInit(): Promise<void> {
    const resp = await fetch(environment.apiRoot + '/sensorconfig', {
      method:'GET'
    })

    if(!resp.ok) {
      console.error('unable to load sensor config', resp)
    } else {
      this.sensorConfig = await resp.json()
      this.configChanged = false
    }
  }

  public ampChanged(amp: number) {
    this.sensorConfig.amp = amp
    this.configChanged = true
  }

  public durationChanged(duration: number) {
    this.sensorConfig.duration = duration
    this.configChanged = true
  }

  public sampleRateChanged(sampleRate: number) {
    this.sensorConfig.sample_rate = sampleRate
    this.configChanged = true
  }

  public dropRateChanged(dropRate: number) {
    this.sensorConfig.drop_rate = dropRate
    this.configChanged = true
  }

  async saveConfig() {
    if(!this.configChanged) {
      return
    }
    const resp = await fetch(environment.apiRoot + '/sensorconfig', {
      method:'POST',
      body: JSON.stringify(this.sensorConfig)
    })
    if(!resp.ok) {
      console.error('unable to save configuration', resp)
    }
    this.configChanged = false
  }

}
