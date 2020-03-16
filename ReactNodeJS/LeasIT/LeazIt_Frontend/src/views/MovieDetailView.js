"use strict";

import React from 'react';

import { ItemDetail } from '../components/ItemDetail';

import MovieService from '../services/MovieService';


export class MovieDetailView extends React.Component {

    constructor(props) {
        super(props);
    }

    componentWillMount(props){
        this.setState({
            loading: true
        });

        let id = this.props.match.params.id;
        console.log(id);
        MovieService.getItem(id).then((data) => {
            this.setState({
                movie: data,
                loading: false
            });
        }).catch((e) => {
            console.error(e);
        });

    }

    deleteMovie(id) {
        MovieService.deleteItem(id).then((message) => {
            this.props.history.push('/');
        }).catch((e) => {
            console.log(e);
        });
    }

    render() {
        if (this.state.loading) {
            return (<h2>Loading...</h2>);
        }

        return (
            <ItemDetail movie={this.state.movie} onDelete={(id) => this.deleteMovie(id)}/>
        );
    }
}
