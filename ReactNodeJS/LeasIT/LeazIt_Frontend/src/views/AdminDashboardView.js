"use strict";

import React from 'react';

import { ItemDetail } from '../components/ItemDetail';

import MovieService from '../services/MovieService';
import {AdminDashboard} from "../components/AdminDashboard";


export class AdminDashboardView extends React.Component {

    constructor(props) {
        super(props);
    }

    componentWillMount(props) {
        this.setState({
            loading: false
        });
    }

    render() {
            if (this.state.loading) {
                return (<h2>Loading...</h2>);
            }
        return (
            <AdminDashboard data={this.props}/>
        );
    }
}
