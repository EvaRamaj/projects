"use strict";

import React, { Component } from 'react';
import HomePage from '../components/HomePage';
import HomePageRP from '../components/HomePageRP';

export class HomePageView extends Component {
    constructor(props) {
        super(props);
    }

    componentWillMount(){
    }

    render() {
        return (
            <HomePageRP/>
        );
    }
}
